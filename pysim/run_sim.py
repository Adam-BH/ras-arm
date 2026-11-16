"""ras-arm physics simulation (PyBullet): the design book's arm, the workcell and the firmware's state machine.

    python run_sim.py                     interactive: buttons in the window, protocol console in the terminal
    python run_sim.py cycle  --cubes 6    the RUN cycle: a feeder puts cubes in the cradle, the arm sorts them
    python run_sim.py vision --cubes 5    overhead camera + OpenCV find loose cubes, the arm sorts them
    python run_sim.py slides [--file my_slides.json]    keyframe sequence (edit ras_sim/slides.py DEMO)
    python run_sim.py demo                cycle, then vision, then slides
    python run_sim.py urdf out.urdf       write the generated arm URDF

    python run_sim.py --serial tcp:5005   + the board's serial port: socket://localhost:5005 (pyserial, pio monitor)
    python run_sim.py --serial COM20      + on a COM port (one end of a com0com pair: the web app opens the other)
    python run_sim.py --serial tcp:5005 --headless     no window: a "board on the desk", until Ctrl-C

Options: --headless (no window, as fast as possible, exit code 1 if a cube ends in the wrong place),
--profile design|firmware (design book constants, default; or config.h as it is), --no-reset (opening the
serial port does not reset the board), --speed 2 (x real time), --seed N, --fail-pick (the first grasp
slips), --log telemetry.csv, --no-camera-window, --quiet.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))


def empty_bins(sim, say):
    """the operator empties both bins between demo phases (35 mm cubes dropped on one spot build a tower
    that the next drop hits); only cubes inside a bin go, and they are tallied for the report"""
    for b in list(sim.world.cubes):
        w = sim.world.where(b)
        if w.startswith("bin"):
            k = w[-1]
            sim.emptied[k] += 1
            sim.emptied[k + "_wrong"] += (k == "A") != (sim.world.cubes[b] == "RED")
            sim.world.remove_cube(b)
    say(f"# operator empties the bins (so far: A {sim.emptied['A']}, B {sim.emptied['B']})")


def scenario(sim, args, cam, say):
    """the chosen demo as one host script"""
    from ras_sim.app import wait_until
    from ras_sim.feeder import feeder
    from ras_sim.slides import DEMO, load_slides, play
    from ras_sim.sorter import SortReport, scatter, vision_sort

    yield from wait_until(sim, lambda: sim.fw.state in ("IDLE", "ERROR"), 60000, "homing")
    if sim.fw.state != "IDLE":
        raise RuntimeError("homing failed")
    results = {}
    if args.mode in ("cycle", "demo"):
        say(f"# RUN cycle: feeding {args.cubes} cubes into the cradle")
        if args.fail_pick:
            sim.world.fail_next_pick()
        sim.send("RUN")
        colours = yield from feeder(sim, args.cubes, seed=args.seed)
        yield from wait_until(sim, lambda: sim.world.cradle_cube() is None and sim.fw.state == "IDLE"
                              or sim.fw.state == "ERROR", 120000, "the last cube")
        sim.send("STOP")
        results["cycle"] = {"fed": colours, "cycles": sim.fw.cycles, "state": sim.fw.state}
        say(f"# cycle done: {sim.fw.cycles} cycles, bins {sim.world.bin_counts()}")
    if args.mode in ("vision", "demo"):
        if args.mode == "demo":
            empty_bins(sim, say)
        placed = scatter(sim, args.cubes, seed=args.seed)
        say(f"# vision sort: {len(placed)} cubes scattered on the table")
        if args.fail_pick and args.mode == "vision":
            sim.world.fail_next_pick()
        rep = SortReport()
        yield from vision_sort(sim, cam, rep, on_look=lambda r: None)
        results["vision"] = rep
        say(f"# vision sort: {rep.text}; sorted {rep.sorted}, slips {rep.slips}, skipped {rep.skipped}")
    if args.mode in ("slides", "demo"):
        if args.mode == "demo":
            empty_bins(sim, say)
        slides = load_slides(args.file) if args.file else DEMO
        if sim.world.cradle_cube() is None:
            sim.world.place_in_cradle("RED")
        yield from play(sim, slides, on_slide=lambda i, n, s: say(f"# slide {i}/{n}: {s.label}"))
        results["slides"] = len(slides)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", default="interactive",
                    choices=["interactive", "cycle", "vision", "slides", "demo", "urdf"])
    ap.add_argument("out", nargs="?", help="urdf: output file")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--profile", choices=["design", "firmware"], default=os.environ.get("RAS_SIM_PROFILE", "design"))
    ap.add_argument("--serial", metavar="tcp:PORT|COMx", help="expose the board's serial port")
    ap.add_argument("--no-reset", action="store_true", help="opening the serial port does not reset the board")
    ap.add_argument("--cubes", type=int, default=6)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--fail-pick", action="store_true")
    ap.add_argument("--file", help="slides: JSON keyframes")
    ap.add_argument("--log", help="write the 20 Hz telemetry to this CSV")
    ap.add_argument("--no-camera-window", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="only events, errors and the report")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    os.environ["RAS_SIM_PROFILE"] = args.profile          # read by ras_sim.config at import

    from ras_sim.app import Simulation
    from ras_sim.config import FW, PROFILE
    from ras_sim.robot_urdf import write_arm_urdf
    from ras_sim.vision import OverheadCamera

    if args.mode == "urdf":
        out = write_arm_urdf(Path(args.out or "ras_arm.urdf"))
        print(f"wrote {out}")
        return 0

    t0 = time.perf_counter()
    gui_obj = None

    def say(line: str, ms: int | None = None):
        if gui_obj:
            return gui_obj.on_line(sim.now if ms is None else ms, line)
        if args.quiet and not line.startswith(("EV ", "ERR ", "#")):
            return
        print(f"{(sim.now if ms is None else ms) / 1000:8.2f}s  {line}", flush=True)

    serve_only = args.headless and args.serial and args.mode == "interactive"
    sim = Simulation(gui=not args.headless, seed=args.seed, echo=lambda ms, ln: say(ln, ms),
                     realtime=not args.headless or serve_only, speed=args.speed)
    sim.emptied = {"A": 0, "B": 0, "A_wrong": 0, "B_wrong": 0}
    print(f"profile {PROFILE}: L1 {FW.L1:g}, L2 {FW.L2:g}, L3 {FW.L3:g}, shoulder {FW.H_SHOULDER:g} mm, "
          f"grip z {FW.CELL_GRIP_Z:g}, drop z {FW.CELL_DROP_Z:g}", flush=True)
    bridge = None
    if args.serial:
        from ras_sim.serial_bridge import SerialBridge
        bridge = SerialBridge(sim, args.serial, reset_on_open=not args.no_reset)
        print(f"# {bridge.describe()}", flush=True)
    log = None
    if args.log:
        log = open(args.log, "w", encoding="utf-8")
        log.write("T,ms,state,j1,j2,j3,j4,j5,j6,grip,x,y,z,ir,colour,err,limits,estop\n")
        sim.tel_sinks.append(lambda ln: log.write(ln + "\n"))

    cam = OverheadCamera(sim.world.cid)
    task = None
    if args.mode != "interactive":
        task = sim.start(scenario(sim, args, cam, say), args.mode)

    try:
        if serve_only:
            print("# serving the serial port in real time, Ctrl-C to stop", flush=True)
            while True:
                sim.tick()
        elif args.headless:
            if task is None:
                print("interactive mode needs the window (drop --headless), or --serial to serve the port")
                return 2
            sim.run_until(lambda: task.done, 30 * 60_000)
        else:
            from ras_sim.gui import Gui
            gui_obj = Gui(sim, camera_window=not args.no_camera_window, bridge=bridge)
            gui_obj.loop()
    except KeyboardInterrupt:
        pass
    finally:
        if log:
            log.close()
        if bridge:
            bridge.close()

    if task is None:
        sim.close()
        return 0
    bins = {k: v + sim.emptied[k] for k, v in sim.world.bin_counts().items()}
    lost = [sim.world.where(b) for b in sim.world.cubes if not sim.world.where(b).startswith("bin")]
    wrong = bins["A_wrong"] + bins["B_wrong"]
    print("\n" + "=" * 64)
    print(f" {args.mode}: {'done' if task.done else 'still running'}"
          + (f"  ERROR: {task.error}" if task.error else ""))
    print(f" bins: A (red) {bins['A']}  B (blue) {bins['B']}  wrong bin {wrong}  not in a bin {len(lost)} {lost or ''}")
    print(f" firmware: {sim.fw.state}, {sim.fw.cycles} cycles, events: "
          f"{sum(1 for ln in sim.log if ln.line.startswith('EV PICK_RETRY'))} pick retries, "
          f"{sum(1 for ln in sim.log if ln.line.startswith('EV ERROR'))} errors")
    print(f" {sim.now / 1000:.1f} s simulated in {time.perf_counter() - t0:.1f} s")
    print("=" * 64)
    sim.close()
    ok = task.done and not task.error and wrong == 0 and not lost and sim.fw.state != "ERROR"
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
