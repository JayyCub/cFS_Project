# Dragon 2 RCS — Sim Thruster Reference

See [DEV_REFERENCE.md](DEV_REFERENCE.md) for how this geometry feeds the control allocator (`ThrusterAllocator.cs`) and the cFS control law.

> **Note:** Thruster positions and directions were read directly from Scene2.unity.
> All force/torque values are verified by the cross-product math and are given at
> **unit thrust** (multiply by the current `ThrusterForce` — 400 N — for actual
> Newtons/N·m).  Only T04–T15 are relevant for ISS docking.  T00–T03 are pure
> retrograde thrusters used for orbital deorbit burns only and are disregarded here.

---

## Body frame

| Axis | Direction |
|------|-----------|
| +Z | Toward ISS (docking / forward) |
| −Z | Away from ISS (retrograde / aft) |
| +X | Right |
| +Y | Up |

Pods are labeled **as seen from the ISS docking port looking toward the capsule**
(i.e. you are at the ISS, looking at the Dragon nose cone):

| Pod | Position |
|-----|----------|
| NE  | Upper-right (+X, +Y) |
| NW  | Upper-left  (−X, +Y) |
| SW  | Lower-left  (−X, −Y) |
| SE  | Lower-right (+X, −Y) |

---

## Thruster groups

There are **three thrusters per pod corner** — one from each group:

| Group | T# | Role |
|-------|----|------|
| **Approach** | T04–T07 | All canted forward (+Z). Pure approach engines. |
| **Brake-Yaw** | T08–T11 | Canted strongly outward and aft (−Z). Primary yaw authority. |
| **Brake-Pitch** | T12–T15 | Canted outward and aft (−Z). Primary pitch authority. |

Corner-to-thruster map:

| Corner | Approach | Brake-Yaw | Brake-Pitch |
|--------|----------|-----------|-------------|
| NE     | T04      | T08       | T12         |
| NW     | T05      | T09       | T13         |
| SW     | T06      | T10       | T14         |
| SE     | T07      | T11       | T15         |

---

## Per-thruster table

All values from scene geometry.  Thrust direction = direction the capsule is pushed.

| # | Corner | Position (x, y, z) | Thrust vector | Primary effect |
|---|--------|---------------------|---------------|----------------|
| T04 | NE | (1.10, 1.66, −3.36) | (−0.22, −0.50, +0.84) | Approach (+Z), slight −X −Y |
| T05 | NW | (−1.10, 1.66, −3.36) | (+0.22, −0.50, +0.84) | Approach (+Z), slight +X −Y |
| T06 | SW | (−1.10, −1.66, −3.36) | (+0.22, +0.50, +0.84) | Approach (+Z), slight +X +Y |
| T07 | SE | (1.10, −1.66, −3.36) | (−0.22, +0.50, +0.84) | Approach (+Z), slight −X +Y |
| T08 | NE | (0.87, 1.77, −3.29) | (−0.70, −0.17, −0.70) | Brake (−Z), +Yaw, −Pitch |
| T09 | NW | (−0.87, 1.77, −3.29) | (+0.70, −0.17, −0.70) | Brake (−Z), −Yaw, −Pitch |
| T10 | SW | (−0.87, −1.77, −3.29) | (+0.70, +0.17, −0.70) | Brake (−Z), −Yaw, +Pitch |
| T11 | SE | (0.87, −1.77, −3.29) | (−0.70, +0.17, −0.70) | Brake (−Z), +Yaw, +Pitch |
| T12 | NE | (1.04, 1.60, −3.05) | (+0.43, −0.50, −0.75) | Brake (−Z), −Pitch, +X |
| T13 | NW | (−1.04, 1.60, −3.05) | (−0.43, −0.50, −0.75) | Brake (−Z), −Pitch, −X |
| T14 | SW | (−1.04, −1.60, −3.05) | (−0.43, +0.50, −0.75) | Brake (−Z), +Pitch, −X |
| T15 | SE | (1.04, −1.60, −3.05) | (+0.43, +0.50, −0.75) | Brake (−Z), +Pitch, +X |

---

## Maneuver firing table

All F and τ values verified.  Zero entries are exact.

### Axial (cleanest — zero torque coupling)

| Maneuver | Thrusters | Net force | Net torque |
|----------|-----------|-----------|------------|
| Approach (+Z) | T04 T05 T06 T07 | (0, 0, +3.35) | 0 |
| Brake — light (−Z) | T08 T09 T10 T11 | (0, 0, −2.78) | 0 |
| Brake — hard (−Z) | T12 T13 T14 T15 | (0, 0, −3.00) | 0 |
| Brake — max (−Z) | T08 T09 T10 T11 T12 T13 T14 T15 | (0, 0, −5.78) | 0 |

### Rotation (clean torque axis, coupled −Z braking)

| Maneuver | Thrusters | τ axis | Coupled force |
|----------|-----------|--------|---------------|
| +Yaw (nose right) | T08 T11 | τ_Y = +5.79 | (−1.39, 0, −1.39) |
| −Yaw (nose left) | T09 T10 | τ_Y = −5.79 | (+1.39, 0, −1.39) |
| +Pitch (nose up) | T10 T11 T14 T15 | τ_X = +9.06 | (0, +1.35, −2.89) |
| −Pitch (nose down) | T08 T09 T12 T13 | τ_X = −9.06 | (0, −1.35, −2.89) |
| +Roll (Z key) | T08 T10 T13 T15 | τ_Z = +4.58 | (0, 0, −2.89) |
| −Roll (X key) | T09 T11 T12 T14 | τ_Z = −4.58 | (0, 0, −2.89) |

> All rotation maneuvers produce a coupled −Z force (braking toward ISS).  The
> allocator compensates by blending in approach thrust from T04–T07.

### Lateral translation (all coupled — allocator resolves)

Because all docking thrusters are aft-mounted, any lateral impulse comes with
a coupled axial component.  The approach group (T04–T07) has the cleanest lateral
components; the allocator pairs them with braking thrusters to cancel the Z coupling.

| Maneuver | Primary thrusters | Lateral force | Coupled Z |
|----------|------------------|---------------|-----------|
| Translate +X (right) | T05 T06 | F_X = +0.45 | +1.67 (approach) |
| Translate −X (left)  | T04 T07 | F_X = −0.45 | +1.67 (approach) |
| Translate +Y (up)    | T06 T07 | F_Y = +1.00 | +1.67 (approach) |
| Translate −Y (down)  | T04 T05 | F_Y = −1.00 | +1.67 (approach) |

---

## TODO

- [ ] Extend scheme to the 4 orbital thrusters (T00–T03) if ever needed for reference
- [ ] Verify cant angles against published Dragon 2 photos/diagrams
- [ ] Confirm roll sign convention (+Z torque = CW or CCW from pilot seat?)
