using UnityEngine;

/// <summary>
/// Simulated Dragon crew hand controllers — the cabin hardware, not flight software.
/// It only reports stick deflections; UdpTelemetrySender sends them to cFS in every
/// SIM_STATE, and GNC flies the vehicle from them (MANUAL phase: translation =
/// acceleration command, rotation = rate command with rate hold). Nothing here touches
/// thrusters — keyboard flight goes through FSW like the real vehicle's does.
///
/// Keyboard mapping (same keys the old Unity-side keyboard RCS used), body frame
/// +Z = toward ISS, +X = right, +Y = up:
///   THC (translation):  W/S = ±Z   D/A = ±X   Space/LeftCtrl = ±Y
///   RHC (rotation):     R/F = ±pitch (X)   E/Q = ±yaw (Y)   Z/X = ±roll (Z)
///
/// Any deflection hands control to the crew (GNC → MANUAL); send GO from the ground
/// console to give it back to the autopilot.
/// </summary>
public class HandController : MonoBehaviour
{
    /// <summary>Translation hand controller deflection, body X/Y/Z, each -1..+1.</summary>
    public Vector3 Translation { get; private set; }

    /// <summary>Rotation hand controller deflection (pitch X, yaw Y, roll Z), each -1..+1.</summary>
    public Vector3 Rotation { get; private set; }

    /// <summary>Re-read the controls. Called by UdpTelemetrySender at each GNC cycle boundary.</summary>
    public void Sample()
    {
        Translation = new Vector3(
            Axis(KeyCode.D, KeyCode.A),
            Axis(KeyCode.Space, KeyCode.LeftControl),
            Axis(KeyCode.W, KeyCode.S));

        Rotation = new Vector3(
            Axis(KeyCode.R, KeyCode.F),
            Axis(KeyCode.E, KeyCode.Q),
            Axis(KeyCode.Z, KeyCode.X));
    }

    static float Axis(KeyCode positive, KeyCode negative)
    {
        float v = 0f;
        if (Input.GetKey(positive)) v += 1f;
        if (Input.GetKey(negative)) v -= 1f;
        return v;
    }
}
