#!/usr/bin/env python3
"""ik_reference.py — reference IK + station validation (float64) → golden test vectors.

Writes the SAME vectors to:
  firmware/test/test_arm/ik_vectors.h        (C IK must match within 0.1 mm / 0.05°)
  sim/src/core/__tests__/ik_vectors.json     (TS IK must match within 0.1 mm / 0.05°)
so firmware and simulator are proven to compute the same angles.
Station validation (firmware/src/4_motion/stations.c ≡ sim/src/core/stations.ts) the same way:
  firmware/test/test_arm/station_vectors.h, sim/src/core/__tests__/station_vectors.json

Conventions (docs/kinematics.md): x forward (cradle), y left, z up, origin = base axis at table.
J1 yaw (0 = cradle), J2 from vertical (+ forward), J3 relative to the upper arm (0 = straight),
J5 = 180 − J2 − J3 keeps the gripper vertical, J6 = roll − J1, J4 = 0.
Run from the repo root:  python3 tools/ik_reference.py
"""
import json, math, os

L1, L2, L3, H = 165.0, 165.0, 110.0, 120.0
COS3_MAX, COS3_MIN = 0.995, -1.0
JMIN = [-100, -20, 20, -90, -100, -90]
JMAX = [100, 110, 150, 90, 100, 90]
OK, UNREACHABLE, LIMIT = 0, 1, 2
MARGIN_COS, MARGIN_DEG = 2e-3, 0.1      # skip borderline cases (float32 vs float64)


def ik(x, y, z, roll):
    """returns (status, [j1..j6], borderline)"""
    r = math.hypot(x, y)
    zw = z + L3 - H
    c3 = (r * r + zw * zw - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    border = abs(c3 - COS3_MAX) < MARGIN_COS or abs(c3 - COS3_MIN) < MARGIN_COS
    if c3 < COS3_MIN or c3 > COS3_MAX:
        return UNREACHABLE, [0.0] * 6, border
    j3 = math.acos(c3)
    j2 = math.atan2(r, zw) - math.atan2(L2 * math.sin(j3), L1 + L2 * math.cos(j3))
    j1 = math.degrees(math.atan2(y, x))
    j2, j3 = math.degrees(j2), math.degrees(j3)
    q = [j1, j2, j3, 0.0, 180.0 - j2 - j3, roll - j1]
    for v, lo, hi in zip(q, JMIN, JMAX):
        if min(abs(v - lo), abs(v - hi)) < MARGIN_DEG:
            border = True
    status = OK if all(lo - 0.01 <= v <= hi + 0.01 for v, lo, hi in zip(q, JMIN, JMAX)) else LIMIT
    return status, q, border


def fk(q):
    s2, s23 = math.radians(q[1]), math.radians(q[1] + q[2])
    r = L1 * math.sin(s2) + L2 * math.sin(s23)
    zw = L1 * math.cos(s2) + L2 * math.cos(s23)
    y1 = math.radians(q[0])
    return [r * math.cos(y1), r * math.sin(y1), zw + H - L3, q[5] + q[0]]


def vectors():
    out = []
    stations = [(160, 0, 170, 0), (220, 0, 24, 0), (0, 220, 78, 0), (0, -220, 78, 0),
                (220, 0, 110, 0), (0, 220, 130, 0), (0, -220, 130, 0)]
    grid = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z, roll)
            for r in range(60, 380, 40) for a in (-95, -60, -20, 0, 35, 90)
            for z in (-40, 0, 24, 78, 130, 200, 260) for roll in (0, 15)]
    for x, y, z, roll in stations + grid:
        st, q, border = ik(x, y, z, roll)
        if border:
            continue
        out.append({"pose": [round(x, 4), round(y, 4), round(z, 4), roll], "status": st,
                    "joints": [round(v, 5) for v in q],
                    "tip": [round(v, 4) for v in fk(q)] if st != UNREACHABLE else None})
    return out


# ── station validation (keep in sync with config.h / stations.c) ──
J1_WORK = (-90.0, 90.0)
SAFE_Z, LINE_STEP = 110.0, 10.0
Z_MIN_TABLE, R_MIN_BASE, MIN_GAP, BIN_MIN_DROP_Z, ABOVE_DZ = 10.0, 90.0, 80.0, 70.0, 60
PARK, PICK, BIN = 0, 1, 2
E_OK, E_UNREACHABLE, E_SINGULAR, E_LIMIT, E_KEEPOUT, E_TOO_CLOSE = range(6)
CAD = [("HOME", PARK, 160, 0, 170, 0), ("CUBE", PICK, 220, 0, 24, 0),
       ("BIN_A", BIN, 0, 220, 78, 90), ("BIN_B", BIN, 0, -220, 78, -90)]
EPS = 0.05                                 # skip cases this close to a threshold (float32 vs float64)


def check(x, y, z, roll, typ):
    """one pose → (reason, borderline)"""
    r = math.hypot(x, y)
    border = abs(z - Z_MIN_TABLE) < EPS or abs(r - R_MIN_BASE) < EPS
    if z < Z_MIN_TABLE or r < R_MIN_BASE:
        return E_KEEPOUT, border
    st, q, b = ik(x, y, z, roll)
    border = border or b
    if st == UNREACHABLE:
        zw = z + L3 - H
        c3 = (r * r + zw * zw - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        border = border or abs(c3 - 1) < MARGIN_COS
        return (E_SINGULAR if COS3_MAX < c3 <= 1 else E_UNREACHABLE), border
    if st == LIMIT:
        return E_LIMIT, border
    if typ != PARK:
        border = border or min(abs(q[0] - J1_WORK[0]), abs(q[0] - J1_WORK[1])) < MARGIN_DEG
        if not J1_WORK[0] <= q[0] <= J1_WORK[1]:
            return E_LIMIT, border
    return E_OK, border


def validate(x, y, z, roll, dz, typ, others):
    """→ (reason, borderline), like stations_validate() for a NEW station (self = -1)"""
    if dz < 0 or (typ == BIN and z < BIN_MIN_DROP_Z):
        return E_KEEPOUT, abs(z - BIN_MIN_DROP_Z) < EPS
    top = z if typ == PARK else max(z + dz, SAFE_Z)
    a, border = z, False
    while True:
        e, b = check(x, y, a, roll, typ)
        border = border or b
        if e:
            return e, border
        if a >= top:
            break
        a = min(a + LINE_STEP, top)
    if typ == PARK:
        return E_OK, border
    for _, t, ox, oy, _, _ in others:
        if t == PARK:
            continue
        d = math.hypot(x - ox, y - oy)
        border = border or abs(d - MIN_GAP) < EPS
        if d < MIN_GAP:
            return E_TOO_CLOSE, border
    return E_OK, border


def station_vectors():
    out = []
    for typ, z in ((BIN, 78), (BIN, 60), (PICK, 24), (PICK, 140), (PARK, 170)):
        for x in range(-100, 341, 20):
            for y in range(-340, 341, 20):
                yaw = round(math.degrees(math.atan2(y, x)))
                for roll in (yaw, 0):
                    e, border = validate(x, y, z, roll, ABOVE_DZ, typ, CAD)
                    if not border:
                        out.append([x, y, z, roll, ABOVE_DZ, typ, e])
    sing = []                                  # the singular shell is thin: scan near full reach, 1 mm steps
    for typ, z in ((PARK, 10), (PARK, 60), (PICK, 10), (BIN, 78)):
        for x in range(280, 331):
            for y in range(0, 200, 7):
                e, border = validate(x, y, z, 0, ABOVE_DZ, typ, CAD)
                if e == E_SINGULAR and not border:
                    sing.append([x, y, z, 0, ABOVE_DZ, typ, e])
    return out + sing[::max(1, len(sing) // 40)]


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    v = vectors()
    js = os.path.join(root, "sim/src/core/__tests__/ik_vectors.json")
    os.makedirs(os.path.dirname(js), exist_ok=True)
    with open(js, "w") as f:
        json.dump(v, f, indent=0)
    h = os.path.join(root, "firmware/test/test_arm/ik_vectors.h")
    with open(h, "w") as f:
        f.write("// GENERATED by tools/ik_reference.py — do not edit. Golden IK vectors (float64 reference).\n")
        f.write("#pragma once\n// {x, y, z, roll, status, j1, j2, j3, j4, j5, j6}\n")
        f.write("static const float IK_VECTORS[][11] = {\n")
        for e in v:
            f.write("  {" + ", ".join(f"{n:.5f}f" for n in e["pose"] + [e["status"]] + e["joints"]) + "},\n")
        f.write("};\n#define IK_N_VECTORS (sizeof IK_VECTORS / sizeof IK_VECTORS[0])\n")
    print(f"{len(v)} vectors ({sum(e['status'] == OK for e in v)} ok) → {h}, {js}")

    sv = station_vectors()
    with open(os.path.join(root, "sim/src/core/__tests__/station_vectors.json"), "w") as f:
        json.dump(sv, f, separators=(",", ":"))
    with open(os.path.join(root, "firmware/test/test_arm/station_vectors.h"), "w") as f:
        f.write("// GENERATED by tools/ik_reference.py — do not edit. Station validation vectors vs the CAD layout.\n")
        f.write("#pragma once\n// {x, y, z, roll, above_dz, type, expected reason}\n")
        f.write("static const int16_t STATION_VECTORS[][7] = {\n")
        for e in sv:
            f.write("  {" + ", ".join(str(n) for n in e) + "},\n")
        f.write("};\n#define STATION_N_VECTORS (sizeof STATION_VECTORS / sizeof STATION_VECTORS[0])\n")
    hist = [sum(e[6] == k for e in sv) for k in range(6)]
    print(f"{len(sv)} station vectors (OK/UNREACH/SING/LIMIT/KEEPOUT/CLOSE = {hist})")


if __name__ == "__main__":
    main()
