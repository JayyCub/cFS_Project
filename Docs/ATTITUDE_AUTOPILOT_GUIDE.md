# Implementing Attitude Autopilot — A Learning Guide

> **Status: historical.** This was written as a self-guided learning exercise *before* the attitude autopilot existed. The work described here is now complete — it shipped as Phase 5F (see [PROJECT.md](PROJECT.md)): the telemetry packet carries per-axis `PitchError_deg`/`YawError_deg`/`RollError_deg`, and Channel 3 of `GNC_APP_ComputeControl()` runs the full PD law (`AttKp`/`MaxAttRate` in the parameter table) described below. Kept as a record of the reasoning process and as a template for tackling the next open item (Phase 6-6, stuck-thruster fault injection) the same way — not as a live TODO.

This guide gives you the map, the right questions, and pointers to the relevant code. It does not give you the implementation. The goal is for you to reason through each decision yourself.

---

## What You're Building

Right now the GNC can null out angular velocity (Channel 3 — rate damping). That means if the chaser is spinning, it fires thrusters to stop the spin. But it has no idea whether the vehicle is *pointed the right way* — it only cares about whether it is *rotating*.

Attitude autopilot means the GNC can command the vehicle to a specific target orientation and hold it there. For docking, the natural target is: **chaser docking port forward = target docking port forward** (ports aligned). The vehicle should autonomously rotate to achieve that alignment and maintain it throughout the approach.

The difference:
- **Rate damping:** "Are you spinning? Stop it." (uses `AngVel` — the derivative term)
- **Attitude control:** "Are you pointed correctly? If not, rotate there. Also, don't spin while doing it." (needs both position error and rate error — a full PD controller)

---

## The Core Problem to Solve

Before writing any code, think through this question:

> *What information does cFS need in order to command a corrective rotation?*

The current `AngVel_X/Y/Z` fields tell cFS how fast the vehicle is rotating around each axis. That is enough for rate damping. But for attitude control you also need to know **how far off** the current attitude is, **on each axis independently**.

Look at what is already in the telemetry packet. Open `gnc_app.h` and find `GNC_APP_UnityTlm_t`. There is already an `AttitudeError_deg` field. Ask yourself:

- Is a single scalar angle enough to command attitude? Why or why not?
- If not, what additional data would give cFS what it needs?
- How many floats would that require? Where in the packet would you add them?

Then open `UdpTelemetrySender.cs` and look at `BuildPacket()`. Notice how the packet is constructed using a byte offset counter. The packet is currently 60 bytes. Any fields you add go on the end, before the close brace of `BuildPacket`. The C struct must be updated to match — look at `GNC_APP_UnityTlm_t` and think about how `__attribute__((packed))` works.

---

## Step 1: Attitude Error in Unity

The telemetry data lives in Unity. Before cFS can control attitude, Unity needs to compute and transmit the right error signal.

**Where to look:**

- `RelativeNav.cs` — find the `attitudeError` property and read exactly how it is computed. It uses `Quaternion.Angle()`. What does that function return? What does it *not* tell you?
- `DockingHUD.cs` — find where roll, pitch, and yaw are displayed. Unity computes this from `transform.rotation`. Read how it decomposes a quaternion into Euler angles.
- `UdpTelemetrySender.cs` — the `chaser` field is a `VehicleState`, which wraps the Rigidbody. You can access `chaser.attitude` (a `Quaternion`) from here.

**Questions to answer before writing code:**

1. What is the *target* attitude for docking? The chaser should be aligned with the target port. Where in the scene is that reference stored? Look at `RelativeNav.cs` — it has references to both `chaserPort` and `targetPort` transforms. How would you use those to define the target attitude?

2. Once you have a target attitude, how do you compute the *error*? The error is the rotation that would take you from your current attitude to your target. In Unity, this is done with an error quaternion. Research `Quaternion.Inverse` and what multiplying two quaternions means geometrically.

3. An error quaternion tells you both the axis of rotation and the angle. How do you extract per-axis error (roll error, pitch error, yaw error in degrees) from it? Look up `Quaternion.ToAngleAxis` and Unity's Euler angle decomposition. Be careful — Euler angles have a frame ambiguity. Decide whether you want errors in the chaser's body frame or the world frame, and why that matters for commanding thrusters.

4. `UdpTelemetrySender` holds a reference to `nav` (a `RelativeNav`). Would it make more sense to compute attitude error here, or add it to `RelativeNav`? Consider where the `chaserPort` and `targetPort` references already live.

---

## Step 2: Extending the Telemetry Packet

Once you know what you want to send, you need to extend the packet in both Unity and cFS.

**Unity side (`UdpTelemetrySender.cs`):**

The `BuildPacket()` method writes floats sequentially. Each `WriteFloat()` advances the offset by 4 bytes. Currently the packet ends with `WriteInt(flags)` at offset 56, giving 60 bytes total.

Think about: where do your new fields go? What byte offset do they start at? What is the new total packet size?

**cFS side (`gnc_app.h`):**

Find `GNC_APP_UnityTlm_t`. Add the corresponding fields to the struct. The order and types must match the order Unity writes them exactly — this is a raw memory copy over UDP, no serialization library.

After changing the struct size, find in `gnc_app_udp.c` where the received packet size is validated (search for the `60` literal). Update it to match the new size. This is a critical coupling point — a mismatch will cause the recv task to silently discard every packet.

---

## Step 3: Upgrading Channel 3 in the Control Law

Open `gnc_app.c` and find `GNC_APP_ComputeControl`. Read Channel 3 — the attitude rate damping block — carefully.

The current logic is:
```
if AngVel_X > 0: fire -pitch thruster, duration = AngVel_X / RotAccel
```

This is a **pure derivative controller** (D-only). It drives rate error to zero. A full PD attitude controller also has a **proportional term** that drives position error (attitude error) to zero:

```
total_error = Kp * attitude_error_axis + Kd * angular_velocity_axis
duration = |total_error| / RotAccel
```

**Questions to think through before changing the code:**

1. What new fields from the telemetry will the proportional term read? These are the fields you added in Step 2.

2. What should `Kp` be? You don't know yet — that is something to tune. But where should it live? Look at `gnc_app_tbl.h` (`GNC_ParamTbl_t`). Should attitude `Kp` be a hardcoded constant, or a table parameter that you can change without recompiling? Think about why the existing gains are in the table and apply the same reasoning.

3. The existing rate damping fires thrusters to counteract angular velocity. The sign convention matters: positive `AngVel_X` fires the negative-pitch thruster (bit 7), not the positive one. When you add the proportional term, the signs must be consistent. Trace through the existing Channel 3 code carefully before adding to it.

4. The total error is the sum of the P and D terms. The sign of the total determines which thruster fires. The magnitude determines the duration. Write out the logic on paper for one axis before touching the code.

5. Should attitude control be active in all phases, or only some? The current rate damping runs in all active phases. Full attitude control might make sense only in APPROACH, or in all phases. Think about what behavior you want in CORRECT and HOLD before deciding.

---

## Step 4: Adding Gains to the Parameter Table

If you decided to put attitude Kp (and possibly Kd) in the parameter table:

1. Add fields to `GNC_ParamTbl_t` in `gnc_app_tbl.h`
2. Add default values in `gnc_param_tbl.c`
3. Read the new fields via `GNC_APP_Data.ParamTblPtr->YourNewField` in `ComputeControl`

The benefit: you can retune without recompiling by uplink-activating a new table image. This is how real flight software handles gain tuning.

---

## Step 5: Verification Strategy

Think about how you will know it is working before committing to the full approach.

**Suggested incremental approach:**

1. **Telemetry first.** Add the new fields, rebuild, and confirm in the EVS wakeup log (`GNC_APP_WAKEUP_INF_EID`) that the attitude error values arriving at cFS are sensible. Manually tilt the chaser in Unity and watch the numbers change. Do not touch the control law yet.

2. **One axis at a time.** Once you trust the data, add the proportional term for pitch only. Tilt the vehicle in pitch (use keyboard R/F in Unity) and watch whether the autopilot corrects it. The HUD shows current roll/pitch/yaw in the bottom-left corner.

3. **Rate damping interaction.** Notice that `RateDamping.cs` (H key) also controls attitude rates locally in Unity, bypassing cFS. When cFS is connected and sending commands, `RateDamping` backs off if any rotation thrusters are active. Make sure you understand this interaction before testing — otherwise you may have two controllers fighting each other.

4. **Check the EVS log.** The `WAKEUP_INF` event prints the commanded thruster mask and duration each second. Watch for oscillation (mask flipping every cycle) — that usually means your Kp is too high or your deadband is too small.

---

## Key Files to Have Open While Working

| Task | File |
|------|------|
| Computing attitude error | `RelativeNav.cs`, `DockingHUD.cs` |
| Sending new fields | `UdpTelemetrySender.cs` (BuildPacket) |
| Receiving new fields | `gnc_app.h` (UnityTlm_t), `gnc_app_udp.c` (size check) |
| Control law | `gnc_app.c` (ComputeControl, Channel 3) |
| New gains | `gnc_app_tbl.h`, `gnc_param_tbl.c` |
| Watching output | EVS console, `DockingHUD.cs` (HUD display) |

---

## Things That Will Trip You Up

**Euler angle ambiguity.** Unity's `eulerAngles` wraps to [0, 360] instead of [-180, 180]. A 350° error and a −10° error are the same rotation. Your error signal needs to handle this or the controller will command the wrong direction for small negative errors. Look at how `DockingHUD.cs` handles this when displaying roll/pitch/yaw.

**Left-handed coordinates.** Unity uses a left-handed coordinate system. Rotations and cross-products behave differently than in a right-handed system. Be careful with sign conventions.

**The 1 Hz update rate.** The GNC fires once per second. If your attitude error is large and Kp is high, the first burn could be very long and overshoot significantly. Start with a small Kp and work up.

**Two controllers in the loop.** The C# `RateDamping` script and the cFS attitude channel both affect angular velocity. The back-off logic in `RateDamping.cs` (it reads `RCSModel.CurrentThrusterMask`) is designed to prevent conflicts, but understand it before assuming it will handle everything automatically.
