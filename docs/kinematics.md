# Kinematics

Code: `firmware/src/4_motion/kinematics.c`, ported line for line to `sim/src/core/kinematics.ts`.
Both are tested against the same float64 reference vectors from `tools/ik_reference.py`
(632 poses, C and TS must agree within 0.05° and 0.1 mm).

## Conventions
- **World frame**: origin on the base axis at table level. **x** forward (toward the cradle), **y** left, **z** up.
- **J1** base yaw: 0 = cradle, +90 = Bin A (+y), −90 = Bin B.
- **J2** upper-arm angle **from vertical**, + = leaning forward.
- **J3** elbow bend **relative to the upper arm**, 0 = straight.
- **J4** forearm roll (0 for picking), **J5** wrist pitch, **J6** wrist roll.
- The gripper is kept **vertical** (pointing down): J2 + J3 + J5 = 180°.
- Lengths: L1 = 180 (shoulder → elbow), L2 = 140 (elbow → wrist), L3 = 110 (wrist → tip), shoulder height H = 120 mm.

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
| HOME | 160, 0, 170 | 0 / 6.8 / 90.9 / 82.3 / 0 |
| CUBE (grip) | 220, 0, 24 | 0 / 47.0 / 93.9 / 39.1 / 0 |
| BIN A drop | 0, 220, 78 | 90 / 35.4 / 88.8 / 55.8 / −90 |
| BIN B drop | 0, −220, 78 | −90 / 35.4 / 88.8 / 55.8 / +90 |

⚠ At the bins J6 sits exactly on its ±90° limit (roll 0 in world yaw). A taught bin pose with a
roll that pushes J6 past ±90° is refused (`ERR … unreachable`); widen the J6 range in `config.h` if the
servo allows it, or teach the bins with roll 0.

## Steps per joint degree
`200 steps × microstep × ratio / 360`: J1 = 200·16/360 ≈ 8.89 · J2 = 200·8·5.18·3/360 ≈ 69.07 ·
J3 = 200·8·5/360 ≈ 22.22. Servos: `µs = centre + value × µs_per_unit` (`config.h`; 270° DS3218 ≈ 7.41 µs/°,
180° MG996 ≈ 11.1 µs/°, gripper 20 µs/mm from 1000 µs closed).

## Straight lines
`motion_line()` splits the segment every 10 mm (max 40 waypoints), checks IK for every waypoint
**before** moving, then feeds them one by one (no array: 2 KB RAM).
