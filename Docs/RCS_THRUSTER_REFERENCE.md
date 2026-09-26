# Dragon 2 RCS — Sim Thruster Reference

**Where thruster data lives (since realism phase 2):**

| Role | Location |
|---|---|
| Hardware, the "real" thrusters | `Scene2.unity`: `ChaserVehicle` → `RCSModel.thrusterTransforms`, `thrusterForce` = 400 N |
| Flight software's model of them | `cFS/apps/rcs/fsw/tables/rcs_thr_tbl.c` (`RCS.ThrTbl`), used by the cFS RCS app for allocation and PWM |

After moving or re-canting a thruster in the scene, run RCSModel's **"Log cFS thruster table (rcs_thr_tbl.c)"** context-menu item and paste its rows into the table. Any difference between the two is effectively a thruster-misalignment error that flight software has to fly through. Run `ThrusterDiagnostic` (F8) to confirm the hardware does what its geometry predicts.

Values below were extracted from the scene on 2026-09-26, after the 2026-07-25 cant fix. They're in the RCSModel body frame, relative to the CoM (override (0, 0, −4) local). All force and torque values are at **unit thrust**; multiply by 400 N for newtons. Only T04–T15 are used for docking. T00–T03 are aft-facing deorbit thrusters, disabled in the RCS table.

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

Thrust direction is the direction the capsule is pushed. Positions are relative to the CoM.

| # | Corner | Position (x, y, z) m | Thrust direction | Enabled |
|---|--------|---------------------|------------------|---------|
| T00–T03 | aft | (±0.68, ±0.40, +3.59) | (0, 0, −1) | no (deorbit) |
| T04 | NE | (+1.104, +1.656, +0.640) | (−0.224, −0.500, +0.837) | yes |
| T05 | NW | (−1.104, +1.656, +0.640) | (+0.224, −0.500, +0.837) | yes |
| T06 | SW | (−1.104, −1.656, +0.640) | (+0.224, +0.500, +0.837) | yes |
| T07 | SE | (+1.104, −1.656, +0.640) | (−0.224, +0.500, +0.837) | yes |
| T08 | NE | (+0.869, +1.769, +0.710) | (−0.707, 0, −0.707) | yes |
| T09 | NW | (−0.869, +1.769, +0.710) | (+0.707, 0, −0.707) | yes |
| T10 | SW | (−0.869, −1.769, +0.710) | (+0.707, 0, −0.707) | yes |
| T11 | SE | (+0.869, −1.769, +0.710) | (−0.707, 0, −0.707) | yes |
| T12 | NE | (+1.038, +1.600, +0.951) | (+0.433, −0.500, −0.750) | yes |
| T13 | NW | (−1.038, +1.600, +0.951) | (−0.433, −0.500, −0.750) | yes |
| T14 | SW | (−1.038, −1.600, +0.951) | (−0.433, +0.500, −0.750) | yes |
| T15 | SE | (+1.038, −1.600, +0.951) | (+0.433, +0.500, −0.750) | yes |

---

## How firings are chosen

No thruster combinations are hard-coded anywhere. Each GNC cycle the cFS RCS app:

1. Takes GNC's requested impulse: linear P (N·s) and angular L (N·m·s), body frame.
2. Solves **non-negative least squares** for per-thruster impulses J ≥ 0. The columns are each thruster's unit-thrust [direction; position × direction]. Thrusters are push-only, so solving with the constraint keeps the cancellation between thrusters exact. When a request is infeasible it returns the closest achievable impulse.
3. Sets each on-time to J ÷ 400 N. If any on-time would exceed `MaxOnTime_s` (0.19 s), it scales the **whole** solution down, preserving direction.
4. Enforces the minimum impulse bit (`MinOnTime_s` = 20 ms, 8 N·s). Pulses of at least half the bit are rounded up to it; smaller ones are dropped.

Because each group is canted, pure translations need several groups firing for different lengths of time. For example, pure +Z uses T04–T07 equally; pure −Z uses the brake groups; lateral moves mix both, with on-times picked so the torques cancel. RCS HK reports `SaturatedCount`, `MinImpulseDrops/Rounds`, `LastResidualNorm` and `LastFiredMask` for every request.

---

## TODO

- [ ] Verify cant angles against published Dragon 2 photos/diagrams
- [ ] Confirm roll sign convention (+Z torque = CW or CCW from pilot seat?)
- [ ] Priority allocation: attitude before translation when a request saturates (today the whole solution scales uniformly)
