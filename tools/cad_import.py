#!/usr/bin/env python3
"""cad_import.py — the arm's CAD → the ROS 2 twin: one STL per link + mass properties.

The CAD comes in the zero pose (every joint 0: arm straight up, fingers up, facing +x = the cradle)
(ros2/README.md, "CAD"). Two ways in:

  cad_import.py list  ARM.step               parts: name, size, centre, suggested link, overall extent
  cad_import.py map   ARM.step > map.yaml    first-guess part → link map, to check and edit
  cad_import.py build ARM.step map.yaml      meshes/<link>.stl + config/cad_meshes.yaml
  cad_import.py stl   DIR                    DIR/<link>.stl already exported per link (binary or ASCII, mm)
  cad_import.py check                        the imported meshes: units, origin, pose
  cad_import.py selftest                     a named STEP assembly of the placeholder arm → round trip

Options, when the CAD is not already in the robot frame (origin on the J1 axis at table level, z up):
  --up y           the export is Y-up (Fusion 360's default)
  --origin X,Y,Z   where the J1 axis meets the table, in CAD coordinates (mm)
  --yaw DEG        then turn about the vertical so the arm faces +x (the cradle)

STEP needs OpenCASCADE for Python (only for STEP; `stl` and `check` need numpy + PyYAML only):
  python3 -m venv --system-site-packages ~/.venvs/cad && ~/.venvs/cad/bin/pip install cadquery-ocp
  ~/.venvs/cad/bin/python tools/cad_import.py list ARM.step
Part names come from the STEP occurrences (Fusion / SolidWorks: "Motor:2"); the map matches them with
regular expressions, plus `near: [x, y, z]` for parts that share a name. Display meshes: OpenCASCADE's
tessellation (0.2 mm chord). Mass properties: each link's mass from hardware.yaml, spread like the real
solids (uniform density, exact volume integrals for STEP, triangle integrals for STL), centre of mass
and inertia in the link frame. Units: STL in mm, YAML in m / kg.
"""
import math
import os
import re
import shutil
import struct
import sys
import tempfile

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DESC = os.path.join(ROOT, 'ros2/ras_arm_description')
LINKS = ['base_link', 'turret', 'upper_arm', 'forearm', 'forearm_roll', 'wrist', 'tool', 'finger_left', 'finger_right']
FINGER, BEYOND = 44.0, 7.0                                 # mm, placeholder fingers: length, reach past the tip
FRAME = {'up': 'z', 'origin': np.zeros(3), 'yaw': 0.0}    # --up / --origin / --yaw


def load(name):
    with open(os.path.join(DESC, 'config', name)) as f:
        return yaml.safe_load(f)


def zero_pose():
    """each link frame's position (mm) in the exported pose, and H, L1, L2, L3 (mm)"""
    g, off = load('arm.yaml')['geometry'], load('cad.yaml')['offsets']
    H, L1, L2, L3 = (g[k] * 1000 for k in ('h_shoulder', 'l1', 'l2', 'l3'))
    d4, d6, q0 = off['d4'] * 1000, off['d6'] * 1000, off['export_opening'] * 500
    zt = H + L1 + L2 + L3
    origin = {'base_link': (0, 0, 0), 'turret': (0, 0, 0), 'upper_arm': (0, 0, H), 'forearm': (0, 0, H + L1),
              'forearm_roll': (0, 0, H + L1 + d4), 'wrist': (0, 0, H + L1 + L2), 'tool': (0, 0, H + L1 + L2 + d6),
              'finger_left': (0, q0, zt), 'finger_right': (0, -q0, zt)}
    return {k: np.array(v, float) for k, v in origin.items()}, (H, L1, L2, L3)


def suggest(c, size):
    """first guess of the link a part belongs to, from its centre (mm, robot frame) in the zero pose.
    Parts on a joint axis (pulleys, horns, drums) are ambiguous: the map must be checked."""
    _, (H, L1, L2, L3) = zero_pose()
    x, y, z = c
    zE, zW, zT = H + L1, H + L1 + L2, H + L1 + L2 + L3
    if z < 0.15 * H and max(size[:2]) > 100:
        return 'base_link'
    if z < H + 20:
        return 'turret'
    if z < zE - 22:
        return 'upper_arm'
    if z < zE + 22:
        return 'forearm'
    if z < zW - 16:
        return 'forearm_roll'
    if z < zW + 16:
        return 'wrist'
    if z < zT + BEYOND - FINGER or abs(y) < 2:
        return 'tool'
    return 'finger_left' if y > 0 else 'finger_right'


def to_robot(p):
    """CAD coordinates (mm, … × 3) → robot frame: J1-axis point at the table to 0, Y-up → Z-up, yaw"""
    p = np.asarray(p, float) - FRAME['origin']
    if FRAME['up'] == 'y':
        p = np.stack([p[..., 0], -p[..., 2], p[..., 1]], -1)            # rotation +90° about x: +y → +z
    a = math.radians(FRAME['yaw'])
    return np.stack([math.cos(a) * p[..., 0] - math.sin(a) * p[..., 1], math.sin(a) * p[..., 0] + math.cos(a) * p[..., 1], p[..., 2]], -1)


# ───────────── meshes ─────────────
def write_stl(path, tris):
    tris = np.asarray(tris, np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, tris
    with open(path, 'wb') as f:
        f.write(b'ras-arm cad_import'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rec.tobytes())


def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack('<I', data[80:84])[0] if len(data) >= 84 else -1
    if len(data) == 84 + 50 * n:
        rec = np.frombuffer(data[84:], dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')], count=n)
        return rec['v'].astype(float)
    v = [list(map(float, m.group(1).split())) for m in re.finditer(rb'vertex\s+([^\n]+)', data)]   # ASCII STL
    return np.array(v, float).reshape(-1, 3, 3)


def mesh_properties(tris):
    """closed triangle mesh (mm) → volume, centre, inertia about the centre, density 1 (mm units)"""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    v = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6                  # signed tetrahedra from the origin
    V = v.sum()
    if V < 0:
        v, V = -v, -V                                                  # inward-facing triangles
    com = (v[:, None] * (a + b + c)).sum(0) / 4 / V
    s = a + b + c                                                      # tetra second moments (vertex 0 = origin)
    C = (np.einsum('n,ni,nj->ij', v, s, s) + sum(np.einsum('n,ni,nj->ij', v, p, p) for p in (a, b, c))) / 20
    I0 = np.trace(C) * np.eye(3) - C
    return V, com, I0 - V * (com @ com * np.eye(3) - np.outer(com, com))


def link_entry(link, tris, props, origin, mass):
    """props = (V mm³, centre mm, inertia mm⁵) for density 1, or None → from the triangles"""
    V, com, I = props if props else mesh_properties(tris)
    if V <= 1e-6 or np.min(np.linalg.eigvalsh(I)) <= 0:               # open or broken: its bounding box
        lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        e, com = hi - lo, (lo + hi) / 2
        V = float(np.prod(e))
        I = np.diag([e[1] ** 2 + e[2] ** 2, e[0] ** 2 + e[2] ** 2, e[0] ** 2 + e[1] ** 2]) * V / 12
    I = I * 1e-6 * mass / V                                            # mm⁵ · kg/mm³ → kg·m²
    r = lambda x: float(f'{x:.6g}')
    return {'mesh': f'{link}.stl', 'mass': mass, 'com': [r(x) for x in (com - origin) / 1000],
            'inertia': [r(I[0, 0]), r(I[1, 1]), r(I[2, 2]), r(I[0, 1]), r(I[0, 2]), r(I[1, 2])],
            'triangles': int(len(tris)), 'volume_cm3': r(V / 1000)}


def save(per_link, props=None):
    """meshes/<link>.stl + config/cad_meshes.yaml"""
    origin, _ = zero_pose()
    mass = load('hardware.yaml')['mass']
    out = {}
    for link in LINKS:
        if link not in per_link:
            continue
        tris = np.concatenate(per_link[link])
        write_stl(os.path.join(DESC, 'meshes', f'{link}.stl'), tris)
        out[link] = link_entry(link, tris, (props or {}).get(link), origin[link], mass['finger' if link.startswith('finger') else link])
    with open(os.path.join(DESC, 'config', 'cad_meshes.yaml'), 'w', newline='\n') as f:
        f.write('# GENERATED by tools/cad_import.py — do not edit. Mesh + mass properties per link (m, kg).\n')
        yaml.safe_dump({'links': out}, f, sort_keys=False, default_flow_style=None, width=120)
    print(f'→ {len(out)} links: ' + ', '.join(f'{k} ({v["triangles"]} tri)' for k, v in out.items()))
    return out


def check(per_link=None):
    """sanity of the meshes: mm, origin on the J1 axis at the table, zero pose, where each link sits"""
    origin, (H, L1, L2, L3) = zero_pose()
    if per_link is None:
        per_link = {link: [read_stl(os.path.join(DESC, 'meshes', e['mesh']))] for link, e in load('cad_meshes.yaml')['links'].items()}
    if not per_link:
        print('no meshes imported yet')
        return True
    pts = np.concatenate([np.concatenate(t).reshape(-1, 3) for t in per_link.values()])
    lo, hi, top = pts.min(0), pts.max(0), H + L1 + L2 + L3
    problems = []
    if hi[2] - lo[2] < top / 50:
        problems.append(f'the arm is {hi[2] - lo[2]:.2f} tall: exported in metres? (mm expected, ≈ {top:.0f})')
    if abs(lo[2]) > 25:
        problems.append(f'lowest point at z = {lo[2]:.0f} mm: the origin should be at table level (--origin)')
    if hi[2] < 0.8 * top:
        problems.append(f'highest point at z = {hi[2]:.0f} mm, expected ≈ {top:.0f}: Y-up export (--up y), or not in the zero pose?')
    if max(abs(lo[0] + hi[0]), abs(lo[1] + hi[1])) / 2 > 60:
        problems.append(f'centre at x {(lo[0] + hi[0]) / 2:.0f}, y {(lo[1] + hi[1]) / 2:.0f} mm: the origin should be on the J1 axis (--origin)')
    print(f'{"link":<13} {"z from":>8} {"z to":>8} {"x span":>8} {"y span":>8}   joint z')
    for link, tris in per_link.items():
        p = np.concatenate(tris).reshape(-1, 3)
        a, b = p.min(0), p.max(0)
        print(f'{link:<13} {a[2]:8.1f} {b[2]:8.1f} {b[0] - a[0]:8.1f} {b[1] - a[1]:8.1f}   {origin[link][2]:7.1f}')
    for p in problems:
        print('  ✗', p)
    if not problems:
        print('  ✓ mm, origin at the table on the J1 axis, zero pose')
    return not problems


# ───────────── STEP (OpenCASCADE) ─────────────
def occ():
    try:
        import OCP  # noqa: F401
    except ImportError:
        raise SystemExit('STEP needs OpenCASCADE for Python:\n'
                         '  python3 -m venv --system-site-packages ~/.venvs/cad && ~/.venvs/cad/bin/pip install cadquery-ocp\n'
                         '  ~/.venvs/cad/bin/python tools/cad_import.py ...')


def robot_trsf():
    """the same transform as to_robot(), for OpenCASCADE shapes"""
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    t = gp_Trsf()
    t.SetTranslation(gp_Vec(*(-FRAME['origin'])))
    if FRAME['up'] == 'y':
        r = gp_Trsf()
        r.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), math.pi / 2)
        t = r.Multiplied(t)
    if FRAME['yaw']:
        r = gp_Trsf()
        r.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.radians(FRAME['yaw']))
        t = r.Multiplied(t)
    return t


def read_step(path):
    """every part instance of the assembly, located, in the robot frame: [{name, shape, lo, hi, unique}]"""
    occ()
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.collections import Sequence_TDF_Label
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_SOLID
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString('XmlOcaf'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != IFSelect_RetDone or not reader.Transfer(doc):
        raise SystemExit(f'cannot read {path} as STEP')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        a = TDataStd_Name()
        return a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), a) else ''

    raw = []

    def walk(label, loc, path):
        if tool.IsAssembly_s(label):
            comps = Sequence_TDF_Label()
            tool.GetComponents_s(label, comps, False)
            for i in range(1, comps.Length() + 1):
                c, ref = comps.Value(i), TDF_Label()
                tool.GetReferredShape_s(c, ref)
                walk(ref, loc.Multiplied(tool.GetLocation_s(c)), path + [name(c) or name(ref)])
        else:
            raw.append((path + [name(label)], tool.GetShape_s(label).Moved(loc)))

    roots = Sequence_TDF_Label()
    tool.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [name(roots.Value(i))] if tool.IsAssembly_s(roots.Value(i)) else [])

    if len(raw) == 1:                                                    # one flat part: split it into its solids
        segs, shape = raw[0]
        exp, raw = TopExp_Explorer(shape, TopAbs_SOLID), []
        while exp.More():
            raw.append((segs[:-1] + [f'{segs[-1] if segs else "solid"}#{len(raw) + 1}'], exp.Current()))
            exp.Next()

    cleaned = []
    for segs, shape in raw:
        segs = [s for s in segs if s]
        if len(segs) >= 2 and segs[-2].split(':')[0] == segs[-1]:      # ".../Motor:2/Motor" → ".../Motor:2"
            segs = segs[:-1]
        cleaned.append((segs, shape))
    deep = [s for s, _ in cleaned if len(s) > 1]                        # drop the top assembly every part sits in
    top = deep[0][0] if deep and len(deep) == len(cleaned) and all(s[0] == deep[0][0] for s in deep) else None

    trsf, parts = robot_trsf(), []
    for k, (segs, shape) in enumerate(cleaned):
        if top:
            segs = segs[1:]
        shape = shape.Moved(TopLoc_Location(trsf))
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape, box, False, False)
        lo, hi = box.CornerMin(), box.CornerMax()
        parts.append({'name': '/'.join(segs) or f'#{k + 1}', 'shape': shape,
                      'lo': np.array([lo.X(), lo.Y(), lo.Z()]), 'hi': np.array([hi.X(), hi.Y(), hi.Z()])})
    counts = {}
    for p in parts:
        counts[p['name']] = counts.get(p['name'], 0) + 1
    for p in parts:
        p['unique'] = counts[p['name']] == 1 and not p['name'].startswith('#')
    return parts


def tessellate(shape, chord_mm=0.2):
    """OpenCASCADE's display tessellation → triangles (mm), outward; faces it cannot mesh are counted"""
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    BRepMesh_IncrementalMesh(shape, chord_mm, False, 0.35, True)
    out, failed = [], 0
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face, loc = TopoDS.Face(exp.Current()), TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None or tri.NbTriangles() == 0:
            failed += 1
        else:
            tr = loc.Transformation()
            nodes = np.array([[q.X(), q.Y(), q.Z()] for q in (tri.Node(i).Transformed(tr) for i in range(1, tri.NbNodes() + 1))])
            idx = np.array([tri.Triangle(j).Get() for j in range(1, tri.NbTriangles() + 1)]) - 1
            out.append(nodes[idx[:, ::-1] if face.Orientation() == TopAbs_REVERSED else idx])
        exp.Next()
    return (np.concatenate(out) if out else np.zeros((0, 3, 3))), failed


def solid_properties(shapes):
    """exact volume, centre and inertia (about the centre, density 1, mm) of a set of shapes, or None"""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    total = GProp_GProps()
    for s in shapes:
        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(s, g)
        if g.Mass() > 0:
            total.Add(g)
    if total.Mass() <= 1e-6:
        return None                                                      # surfaces only: use the triangles
    c, M = total.CentreOfMass(), total.MatrixOfInertia()
    return total.Mass(), np.array([c.X(), c.Y(), c.Z()]), np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])


# ───────────── map ─────────────
def centre(p):
    return (p['lo'] + p['hi']) / 2


def load_rules(path):
    with open(path) as f:
        rules = [(re.compile(r['match']), r['link'], np.array(r['near'], float) if 'near' in r else None) for r in yaml.safe_load(f)]
    bad = {link for _, link, _ in rules if link not in LINKS + ['skip']}
    if bad:
        raise SystemExit(f'unknown link(s) in the map: {", ".join(sorted(bad))} (links: {", ".join(LINKS)}, skip)')
    return rules


def rule_for(part, rules):
    """first rule whose regex matches the part name (and whose 'near' point, if any, is within 5 mm of its centre)"""
    for rx, link, near in rules:
        if rx.search(part['name']) and (near is None or np.linalg.norm(centre(part) - near) < 5):
            return link
    return None


def map_lines(parts):
    yield '# part (regular expression on the name, first match wins) → link. Guessed from positions: CHECK EVERY LINE.'
    yield '# A servo body belongs to the link it is screwed to, its horn to the next one. "link: skip" leaves a part out.'
    yield '# near: [x, y, z] (mm, robot frame) picks one of several parts with the same name.'
    for p in parts:
        c, s = centre(p), p['hi'] - p['lo']
        near = '' if p['unique'] else f', near: [{c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}]'
        yield (f'- {{match: {yaml.safe_dump("^" + re.escape(p["name"]) + "$", default_style=chr(39)).strip()}{near}, '
               f'link: {suggest(c, s)}}}   # size {s[0]:.0f} × {s[1]:.0f} × {s[2]:.0f}')


# ───────────── commands ─────────────
def cmd_list(step):
    parts = read_step(step)
    print(f'{len(parts)} parts in {os.path.basename(step)} (mm, robot frame)')
    print(f'{"part":<44} {"size x × y × z":>18} {"centre x, y, z":>21}  suggested link')
    for p in parts:
        c, s = centre(p), p['hi'] - p['lo']
        print(f'{p["name"][-44:]:<44} {s[0]:6.0f}{s[1]:6.0f}{s[2]:6.0f} {c[0]:7.0f}{c[1]:7.0f}{c[2]:7.0f}  {suggest(c, s)}')
    lo, hi = np.min([p['lo'] for p in parts], 0), np.max([p['hi'] for p in parts], 0)
    print(f'all parts: x {lo[0]:.0f}…{hi[0]:.0f}, y {lo[1]:.0f}…{hi[1]:.0f}, z {lo[2]:.0f}…{hi[2]:.0f} mm  '
          f'(zero pose: z 0…{sum(zero_pose()[1]):.0f} centred on x = y = 0, else --up / --origin / --yaw)')


def cmd_map(step):
    print('\n'.join(map_lines(read_step(step))))


def cmd_build(step, map_path):
    rules = load_rules(map_path)
    parts = read_step(step)
    per_link, shapes, unmatched, failed = {}, {}, [], 0
    for p in parts:
        link = rule_for(p, rules)
        if link is None:
            unmatched.append(p['name'])
        elif link != 'skip':
            tris, f = tessellate(p['shape'])
            failed += f
            if len(tris):
                per_link.setdefault(link, []).append(tris)
                shapes.setdefault(link, []).append(p['shape'])
    if unmatched:
        print(f'⚠ {len(unmatched)} parts match no rule, left out: {", ".join(unmatched[:12])}{" …" if len(unmatched) > 12 else ""}')
    if failed:
        print(f'⚠ {failed} faces could not be tessellated: small holes in the display meshes')
    ok = check(per_link)
    save(per_link, {link: solid_properties(s) for link, s in shapes.items()})
    return ok


def cmd_stl(folder):
    per_link = {l: [to_robot(read_stl(os.path.join(folder, f'{l}.stl')))] for l in LINKS if os.path.exists(os.path.join(folder, f'{l}.stl'))}
    if not per_link:
        raise SystemExit(f'no <link>.stl in {folder} (links: {", ".join(LINKS)})')
    ok = check(per_link)
    save(per_link)
    return ok


# ───────────── selftest ─────────────
def placeholder_parts():
    """the URDF's placeholder shapes in the zero pose (mm) as CAD parts: (name, link, kind, centre, size).
    DS3218MG is one part used twice, like the real J4 / J5 servos."""
    _, (H, L1, L2, L3) = zero_pose()
    zW, zT = H + L1 + L2, H + L1 + L2 + L3
    return [('BasePlate', 'base_link', 'cyl_z', (0, 0, 4), (80, 8)),
            ('Turntable', 'turret', 'cyl_z', (0, 0, (H + 8) / 2), (62, H - 8)),
            ('ShoulderDrum', 'upper_arm', 'cyl_y', (0, 0, H), (30, 44)),
            ('UpperArm', 'upper_arm', 'box', (0, 0, H + L1 / 2), (40, 40, L1)),
            ('ElbowDrum', 'forearm', 'cyl_y', (0, 0, H + L1), (22, 44)),
            ('DS3218MG', 'forearm', 'box', (0, 30, H + L1 + 30), (20, 40, 40)),
            ('Forearm', 'forearm_roll', 'box', (0, 0, H + L1 + L2 / 2), (32, 32, L2)),
            ('DS3218MG', 'forearm_roll', 'box', (0, 30, zW - 30), (20, 40, 40)),
            ('WristDrum', 'wrist', 'cyl_y', (0, 0, zW), (16, 44)),
            ('GripperBody', 'tool', 'box', (0, 0, zW + (L3 - FINGER + BEYOND) / 2), (26, 26, L3 - FINGER + BEYOND)),
            ('GripperPalm', 'tool', 'box', (0, 0, zW + L3 - FINGER + BEYOND), (20, 70, 8)),
            ('FingerL', 'finger_left', 'box', (0, 4, zT + BEYOND - FINGER / 2), (14, 8, FINGER)),
            ('FingerR', 'finger_right', 'box', (0, -4, zT + BEYOND - FINGER / 2), (14, 8, FINGER))]


def _box(part):
    """bounding box (lo, hi) of a placeholder part, mm"""
    _, _, kind, c, s = part
    c = np.array(c, float)
    half = {'box': np.array(s, float) / 2, 'cyl_z': np.array([s[0], s[0], s[1] / 2]), 'cyl_y': np.array([s[0], s[1] / 2, s[0]])}[kind]
    return np.r_[c - half, c + half]


def write_test_step(path, origin):
    """the placeholder arm as a named STEP assembly, like Fusion exports it by default: Y up, origin elsewhere,
    each part built around its own origin and placed by its occurrence's location"""
    occ()
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
    from OCP.gp import gp_Ax1, gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    doc = TDocStd_Document(TCollection_ExtendedString('XmlOcaf'))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    label = lambda lab, n: TDataStd_Name.Set_s(lab, TCollection_ExtendedString(n))
    assy = tool.NewShape()
    label(assy, 'ras_arm v3')
    to_cad = gp_Trsf()                                                   # robot → CAD: Y up, then the offset origin
    to_cad.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0)), -math.pi / 2)
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(*origin))
    to_cad = shift.Multiplied(to_cad)
    defs, count = {}, {}
    for name, _, kind, c, s in placeholder_parts():
        if name not in defs:
            if kind == 'box':
                shape = BRepPrimAPI_MakeBox(gp_Pnt(-s[0] / 2, -s[1] / 2, -s[2] / 2), s[0], s[1], s[2]).Shape()
            elif kind == 'cyl_z':
                shape = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -s[1] / 2), gp_Dir(0, 0, 1)), s[0], s[1]).Shape()
            else:
                shape = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, -s[1] / 2, 0), gp_Dir(0, 1, 0)), s[0], s[1]).Shape()
            defs[name] = tool.AddShape(shape, False)
            label(defs[name], name)
        place = gp_Trsf()
        place.SetTranslation(gp_Vec(*c))
        count[name] = count.get(name, 0) + 1
        comp = tool.AddComponent(assy, defs[name], TopLoc_Location(to_cad.Multiplied(place)))
        label(comp, f'{name}:{count[name]}')
    tool.UpdateAssemblies()
    w = STEPCAFControl_Writer()
    w.SetNameMode(True)
    sys.stdout.flush()
    saved = os.dup(1)                                                    # OCCT prints a transfer banner on fd 1
    os.dup2(os.open(os.devnull, os.O_WRONLY), 1)
    try:
        w.Transfer(doc, STEPControl_AsIs)
        w.Write(path)
    finally:
        os.dup2(saved, 1)
        os.close(saved)


def cmd_selftest():
    """named Y-up STEP assembly of the placeholder arm → list → map → build: names, frame, geometry, inertia"""
    tmp = tempfile.mkdtemp(prefix='cad_selftest_')
    step, origin = os.path.join(tmp, 'arm.step'), np.array([30.0, 15.0, -40.0])
    write_test_step(step, origin)
    FRAME.update(up='y', origin=origin, yaw=0.0)
    errs, notes = [], []
    parts = read_step(step)
    names = sorted(p['name'] for p in parts)
    want = sorted(f'{n}:{k}' for n, k in [(n, sum(1 for m, *_ in placeholder_parts()[:i + 1] if m == n))
                                          for i, (n, *_) in enumerate(placeholder_parts())])
    if names != want:
        errs.append(f'part names {names} ≠ {want}')
    truth = {}
    for i, (n, link, *_) in enumerate(placeholder_parts()):
        truth[f'{n}:{sum(1 for m, *_ in placeholder_parts()[:i + 1] if m == n)}'] = link
    expect = {}
    for part in placeholder_parts():
        b = _box(part)
        e = expect.setdefault(part[1], [b[:3], b[3:]])
        e[0], e[1] = np.minimum(e[0], b[:3]), np.maximum(e[1], b[3:])
    for p in parts:
        guess = suggest(centre(p), p['hi'] - p['lo'])
        if p['name'] in truth and guess != truth[p['name']]:
            notes.append(f'{p["name"]}: guessed {guess}, is {truth[p["name"]]}')
    rules = os.path.join(tmp, 'map.yaml')
    with open(rules, 'w') as f:                                          # a reviewed map: names, the servos by instance
        yaml.safe_dump([{'match': f'^{re.escape(n)}$', 'link': l} for n, l in truth.items()], f)
    backup = open(os.path.join(DESC, 'config', 'cad_meshes.yaml'), 'rb').read()
    try:
        ok = cmd_build(step, rules)
        got = load('cad_meshes.yaml')['links']
        for link, (lo, hi) in expect.items():
            p = read_stl(os.path.join(DESC, 'meshes', f'{link}.stl')).reshape(-1, 3)
            d = max(np.abs(p.min(0) - lo).max(), np.abs(p.max(0) - hi).max())
            if d > 0.5:
                errs.append(f'{link}: bounding box off by {d:.2f} mm')
            if link not in got:
                errs.append(f'{link}: missing in cad_meshes.yaml')
        # a single box: the finger's centre of mass and inertia are known exactly
        f, m = got['finger_left'], load('hardware.yaml')['mass']['finger']
        x, y, z = 0.014, 0.008, FINGER / 1000
        if np.abs(np.array(f['com']) - [0, 0.004, (BEYOND - FINGER / 2) / 1000]).max() > 1e-6:
            errs.append(f'finger_left centre {f["com"]}')
        if np.abs(np.array(f['inertia'][:3]) / (m / 12 * np.array([y * y + z * z, x * x + z * z, x * x + y * y])) - 1).max() > 1e-3:
            errs.append(f'finger_left inertia {f["inertia"][:3]}')
        print(f'selftest: STEP assembly, {len(parts)} named parts (Y up, origin {origin.tolist()}), '
              f'{len(expect)} links: {"OK" if ok and not errs else "FAILED"}')
        for e in errs:
            print('  ✗', e)
        for n in notes:
            print('  · the map guess was wrong for', n)
        return ok and not errs
    finally:
        with open(os.path.join(DESC, 'config', 'cad_meshes.yaml'), 'wb') as fh:
            fh.write(backup)
        for link in expect:
            p = os.path.join(DESC, 'meshes', f'{link}.stl')
            if os.path.exists(p):
                os.remove(p)
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv):
    cmds = {'list': (cmd_list, 1), 'map': (cmd_map, 1), 'build': (cmd_build, 2), 'stl': (cmd_stl, 1),
            'check': (lambda: check(), 0), 'selftest': (cmd_selftest, 0)}
    args, opts, i = [], {}, 1
    while i < len(argv):
        if argv[i] in ('--up', '--origin', '--yaw') and i + 1 < len(argv):
            opts[argv[i]] = argv[i + 1]
            i += 2
        else:
            args.append(argv[i])
            i += 1
    try:
        FRAME['up'] = opts.get('--up', 'z').lower()
        FRAME['origin'] = np.array([float(v) for v in opts['--origin'].split(',')]) if '--origin' in opts else np.zeros(3)
        FRAME['yaw'] = float(opts.get('--yaw', 0))
        assert FRAME['up'] in ('y', 'z') and FRAME['origin'].shape == (3,)
    except (ValueError, AssertionError):
        print('options: --up y|z  --origin X,Y,Z (mm, CAD frame)  --yaw DEG')
        return 2
    if not args or args[0] not in cmds or len(args) - 1 != cmds[args[0]][1]:
        print(__doc__)
        return 2
    return 1 if cmds[args[0]][0](*args[1:]) is False else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
