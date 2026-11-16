# Kinematics

Code: `firmware/src/4_motion/kinematics.c`, ported line for line to `pysim/ras_sim/kinematics.py`.
Both are tested against the same float64 reference vectors from `tools/ik_reference.py`
(599 poses, C and Python must agree within 0.05° and 0.1 mm).

## Conventions
- **World frame**: origin on the base axis at table level. **x** forward (toward the cradle), **y** left, **z** up.
- **J1** base yaw: 0 = cradle, +90 = Bin A (+y), −90 = Bin B.
- **J2** upper-arm angle **from vertical**, + = leaning forward.
- **J3** elbow bend **relative to the upper arm**, 0 = straight.
- **J4** forearm roll (0 for picking), **J5** wrist pitch, **J6** wrist roll.
- The gripper is kept **vertical** (pointing down): J2 + J3 + J5 = 180°.
- Lengths (design book, adopted design): L1 = 165 (shoulder → elbow), L2 = 190 (elbow → wrist pitch, J4 at 140),
  L3 = 200 (wrist pitch → fingertip: 100 mm link + 100 mm gripper), shoulder height H = 300 mm (base tower).

## Inverse kinematics (tip pose → joints)
Because the tool is vertical, the wrist sits L3 straight above the tip. That leaves a base yaw plus a
2-link planar arm in the (r, z) plane, solved with the law of cosines:

```
r    = √(x² + y²)                         horizontal reach
z_w  = z + L3 − H                         wrist height above the shoulder axis
cos J3 = (r² + z_w² − L1² − L2²) / (2·L1·L2)     reject if > 0.995 (straight arm) or < −1
J3   = acos(cos J3)                       elbow-up solution
J2   = atan2(r, z_w) − atan2(L2·sin J3, L1 + L2·cos J3)
J1   = atan2(y, x)
J5   = 180 − J2 − J3                      keeps the gripper vertical
J6   = roll − J1                          keeps the fingers aligned with the cube in world yaw
J4   = 0
```

Then the joint limits are checked (J1 −100…100, J2 −20…110, J3 20…150, J4 −90…90, J5 −100…100,
J6 −90…90). Result: `IK_OK`, `IK_UNREACHABLE` (outside the annulus) or `IK_LIMIT`.
The gripper value is never touched or judged by IK.

## Forward kinematics (joints → tip)
```
r   = L1·sin J2 + L2·sin(J2 + J3)
z_w = L1·cos J2 + L2·cos(J2 + J3)
x = r·cos J1,  y = r·sin J1,  z = z_w + H − L3,  roll = J6 + J1
```

## Stations (all reachable, tested)
| Pose | x, y, z (mm) | J1 / J2 / J3 / J5 / J6 (deg) |
|---|---|---|
| HOME | 160, 0, 170 | 0 / −1.6 / 121.6 / 60.0 / 0 |
| CUBE (grip, fingertips at the centre of the 35 mm cube on the 20 mm V-cradle) | 220, 0, 37.5 | 0 / 51.0 / 100.1 / 28.9 / 0 |
| BIN A drop | 0, 220, 88 | 90 / 36.2 / 103.6 / 40.2 / −90 |
| BIN B drop | 0, −220, 88 | −90 / 36.2 / 103.6 / 40.2 / +90 |

⚠ At the bins J6 sits exactly on its ±90° limit (roll 0 in world yaw). A taught bin pose with a
roll that pushes J6 past ±90° is refused (`ERR … unreachable`); widen the J6 range in `config.h` if the
servo allows it, or teach the bins with roll 0.

## Steps per joint degree
`200 steps × microstep × ratio / 360`: J1 = 200·16/360 ≈ 8.89 · J2 = 200·8·5.18·5.5/360 ≈ 126.62
(PG518 × belt 16T → 88T) · J3 = 200·8·6.5/360 ≈ 28.89 (belt 16T → 104T). Servos: `µs = centre + value × µs_per_unit` (`config.h`; 270° DS3218 ≈ 7.41 µs/°,
180° MG996 ≈ 11.1 µs/°, gripper 20 µs/mm from 1000 µs closed).

## Straight lines
`motion_line()` splits the segment every 10 mm (max 40 waypoints), checks IK for every waypoint
**before** moving, then feeds them one by one (no array: 2 KB RAM).
