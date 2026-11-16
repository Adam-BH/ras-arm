# Kinematics

Code: `firmware/src/4_motion/kinematics.c`, ported line for line to `sim/src/core/kinematics.ts`.
Both are tested against the same float64 reference vectors from `tools/ik_reference.py`
(619 poses, C and TS must agree within 0.05° and 0.1 mm).

## Conventions
- **World frame**: origin on the base axis at table level. **x** forward (toward the cradle), **y** left, **z** up.
- **J1** base yaw: 0 = cradle, +90 = Bin A (+y), −90 = Bin B.
- **J2** upper-arm angle **from vertical**, + = leaning forward.
- **J3** elbow bend **relative to the upper arm**, 0 = straight.
- **J4** forearm roll (0 for picking), **J5** wrist pitch, **J6** wrist roll.
- The gripper is kept **vertical** (pointing down): J2 + J3 + J5 = 180°.
- Lengths (v2 design): L1 = 165 (shoulder → elbow), L2 = 165 (elbow → wrist), L3 = 110 (wrist → tip), shoulder height H = 120 mm.

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

## CAD default stations (all valid with the v2 lengths, tested)
The layout is data (README → Configurable workcell); these are the first-boot / `ST FACTORY` values.

| Station | x, y, z (mm), roll | J1 / J2 / J3 / J5 / J6 (deg) |
|---|---|---|
| HOME | 160, 0, 170, 0 | 0 / −1.7 / 93.4 / 88.3 / 0 |
| CUBE (grip) | 220, 0, 24, 0 | 0 / 38.3 / 96.2 / 45.6 / 0 |
| CUBE approach | 220, 0, 110 | 0 / 22.6 / 85.8 / 71.5 / 0 |
| BIN_A drop | 0, 220, 78, 90 | 90 / 27.1 / 91.5 / 61.4 / 0 |
| BIN_A approach | 0, 220, 138, 90 | 90 / 20.3 / 79.1 / 80.7 / 0 |
| BIN_B drop | 0, −220, 78, −90 | −90 / 27.1 / 91.5 / 61.4 / 0 |

Bins use roll = ±90 so J6 = roll − J1 = 0 (fingers tangent to the base, mid-range).
⚠ With the v2 lengths **J5 (max 100°) limits high poses**: the tool stays vertical, so J5 = 180 − J2 − J3
grows as the tip rises close to the base. Example: a station at (160, 0, 150) is fine, but its approach
60 mm higher needs J5 ≈ 102° and is refused (`LIMIT`). Keep pick/bin stations low; the reach shading in
the sim shows where they can go.

## Steps per joint degree
`200 steps × microstep × ratio / 360` (`MICROSTEP_Jn`, `RATIO_*` in `config.h`): J1 = 200·16/360 ≈ 8.89 ·
J2 = 200·8·5.18·(88/16)/360 ≈ 126.6 · J3 = 200·8·(104/16)/360 ≈ 28.9. The J1 trapezoid is also capped at
`BASE_MAX_ACC_DPS2` = 143 °/s² (2.5 rad/s²): a 90° base swing takes ≥ 1.83 s. Servos: `µs = centre + value × µs_per_unit` (`config.h`; 270° DS3218 ≈ 7.41 µs/°,
180° MG996 ≈ 11.1 µs/°, gripper 20 µs/mm from 1000 µs closed).

## Straight lines
`motion_line()` splits the segment every 10 mm (max 40 waypoints), checks IK for every waypoint
**before** moving, then feeds them one by one (no array: 2 KB RAM).
