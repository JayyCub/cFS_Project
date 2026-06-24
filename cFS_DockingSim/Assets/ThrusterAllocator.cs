using UnityEngine;

/// <summary>
/// Control allocator for an over-actuated RCS thruster set.
///
/// At startup call Initialize() with the thruster geometry.  It builds the
/// 6×N effectiveness matrix B (columns = per-thruster [force; torque] wrench)
/// and computes the right pseudo-inverse B† = B^T·(B·B^T)^-1, which gives the
/// minimum-energy throttle distribution for any desired 6-DOF wrench.
///
/// Per-call, Allocate() returns a float[N] throttle array.  Thrusters are
/// unidirectional (push-only), so callers should threshold or clamp negative
/// values before using them.
/// </summary>
public class ThrusterAllocator
{
    private float[,] _Bpinv; // [N, 6] right pseudo-inverse
    private int      _n;

    public bool IsReady => _Bpinv != null;

    public void Initialize(ThrusterDef[] thrusters)
    {
        _n     = thrusters.Length;
        _Bpinv = null;

        if (_n == 0)
        {
            Debug.LogWarning("[ThrusterAllocator] No thrusters — allocator disabled.");
            return;
        }

        // B [6, N]: column i = [dir; r×dir] — wrench produced by thruster i at 1 N thrust.
        float[,] B = new float[6, _n];
        for (int i = 0; i < _n; i++)
        {
            Vector3 d = thrusters[i].direction;
            Vector3 t = Vector3.Cross(thrusters[i].position, d);
            B[0, i] = d.x;  B[1, i] = d.y;  B[2, i] = d.z;
            B[3, i] = t.x;  B[4, i] = t.y;  B[5, i] = t.z;
        }

        // BBT = B · B^T [6, 6]
        float[,] BBT = new float[6, 6];
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                for (int k = 0; k < _n; k++)
                    BBT[r, c] += B[r, k] * B[c, k];

        float[,] BBTinv = Invert6x6(BBT);
        if (BBTinv == null)
        {
            Debug.LogWarning("[ThrusterAllocator] B·B^T is singular — check thruster geometry (collinear or missing axes?).");
            return;
        }

        // B† = B^T · BBT^-1 [N, 6]
        _Bpinv = new float[_n, 6];
        for (int i = 0; i < _n; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    _Bpinv[i, j] += B[k, i] * BBTinv[k, j];

        Debug.Log($"[ThrusterAllocator] Ready — {_n} thrusters.");
    }

    /// <summary>
    /// Map a desired body-frame wrench to per-thruster throttle values.
    /// Negative throttles mean the minimum-norm solution wants a "pull" from that
    /// thruster; callers should clamp or ignore negatives (thrusters are push-only).
    /// </summary>
    public float[] Allocate(Vector3 force, Vector3 torque)
    {
        float[] u = new float[_n];
        if (_Bpinv == null) return u;

        float[] w = { force.x, force.y, force.z, torque.x, torque.y, torque.z };
        for (int i = 0; i < _n; i++)
            for (int j = 0; j < 6; j++)
                u[i] += _Bpinv[i, j] * w[j];

        return u;
    }

    // Gauss-Jordan inversion with partial pivoting.  Returns null if singular.
    static float[,] Invert6x6(float[,] m)
    {
        const int N = 6;
        float[,] aug = new float[N, N * 2];

        for (int r = 0; r < N; r++)
            for (int c = 0; c < N; c++)
            {
                aug[r, c]     = m[r, c];
                aug[r, c + N] = (r == c) ? 1f : 0f;
            }

        for (int col = 0; col < N; col++)
        {
            // Partial pivot
            int pivot = col;
            for (int r = col + 1; r < N; r++)
                if (Mathf.Abs(aug[r, col]) > Mathf.Abs(aug[pivot, col]))
                    pivot = r;

            if (Mathf.Abs(aug[pivot, col]) < 1e-9f) return null;

            if (pivot != col)
                for (int c = 0; c < N * 2; c++)
                    (aug[col, c], aug[pivot, c]) = (aug[pivot, c], aug[col, c]);

            float s = aug[col, col];
            for (int c = 0; c < N * 2; c++) aug[col, c] /= s;

            for (int r = 0; r < N; r++)
            {
                if (r == col) continue;
                float f = aug[r, col];
                for (int c = 0; c < N * 2; c++) aug[r, c] -= f * aug[col, c];
            }
        }

        float[,] inv = new float[N, N];
        for (int r = 0; r < N; r++)
            for (int c = 0; c < N; c++)
                inv[r, c] = aug[r, c + N];
        return inv;
    }
}
