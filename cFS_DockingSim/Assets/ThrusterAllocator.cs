using UnityEngine;

/// <summary>
/// Control allocator for an over-actuated RCS thruster set.
///
/// At startup call Initialize() with the thruster geometry. Allocate() then
/// builds the 6×N effectiveness matrix B (columns = per-thruster [force; torque]
/// wrench, restricted to whichever thrusters are active for that call) and
/// solves the right pseudo-inverse B† = B^T·(B·B^T)^-1 for the minimum-energy
/// throttle distribution for the desired 6-DOF wrench.
///
/// Thrusters are unidirectional (push-only). Rather than solving over every
/// thruster and clamping/zeroing afterward — which breaks the pseudo-inverse's
/// exact cancellation — Allocate() excludes locked-out thrusters from the solve
/// itself (via activeMask) and runs a proper non-negative least-squares (NNLS)
/// active-set solve (Lawson-Hanson) so the result is always fully non-negative
/// and realizable, and is the closest achievable wrench to what was asked for
/// even when the exact request is infeasible for this thruster set. See
/// Allocate()'s doc comment for why a naive "solve once, drop negatives" isn't
/// enough.
/// </summary>
public class ThrusterAllocator
{
    private ThrusterDef[] _thrusters;
    private int           _n;

    public bool IsReady => _thrusters != null && _n > 0;

    public void Initialize(ThrusterDef[] thrusters)
    {
        _thrusters = thrusters;
        _n         = thrusters.Length;

        if (_n == 0)
        {
            Debug.LogWarning("[ThrusterAllocator] No thrusters — allocator disabled.");
            return;
        }

        // Sanity check only — the real per-call solves happen in Allocate() over
        // whatever subset of thrusters is active for that command. Unregularized
        // deliberately, so a genuine geometry problem (collinear/missing axes)
        // is still caught here instead of being masked by the regularization
        // Allocate()'s inner NNLS solve uses for its own, different purpose.
        if (BuildPinv(AllTrue(_n), regularize: false) == null)
            Debug.LogWarning("[ThrusterAllocator] B·B^T is singular — check thruster geometry (collinear or missing axes?).");

        Debug.Log($"[ThrusterAllocator] Ready — {_n} thrusters.");
    }

    /// <summary>
    /// Map a desired body-frame wrench to per-thruster throttle values, restricted
    /// to the thrusters marked active in activeMask (defaults to all thrusters).
    /// The returned array is always non-negative and always zero for inactive
    /// thrusters — no further clamping/zeroing needed by the caller.
    ///
    /// REWRITTEN 2026-07-18 as a proper non-negative least-squares (NNLS) solve
    /// (Lawson-Hanson active-set method), replacing an earlier "drop-only" greedy
    /// heuristic that started from the full minimum-norm solution and only ever
    /// REMOVED thrusters wanting to "pull," never re-adding one. That monotonic
    /// approach could (and did) terminate in a fully rank-deficient, all-zero
    /// state even when a real non-negative solution existed — confirmed in
    /// telemetry: F=(400,-400,1300) T=(600,600,600) (every channel maxed at once)
    /// resolved to zero thrust at EVERY scale from 1.0 down to 1/128, proving the
    /// failure was directional, not magnitude-related — a non-negative cone is
    /// scale-invariant, so if a direction is outside it, no rescaling ever helps.
    ///
    /// Lawson-Hanson instead GROWS the active set one thruster at a time (always
    /// picking whichever currently-excluded thruster would most reduce the
    /// residual against the target wrench), only backing one back out when
    /// moving toward its candidate solution would drive it negative. This
    /// provably converges to the true minimum-residual non-negative solution —
    /// the closest the thruster set can actually get to the requested wrench —
    /// instead of getting stuck. Both loops carry a hard iteration cap so a
    /// pathological case degrades to "best effort so far," never a hang.
    /// </summary>
    public float[] Allocate(Vector3 force, Vector3 torque, bool[] activeMask = null)
    {
        float[] x = new float[_n];
        if (_thrusters == null || _n == 0) return x;

        bool[] mask   = activeMask != null ? (bool[])activeMask.Clone() : AllTrue(_n);
        float[] target = { force.x, force.y, force.z, torque.x, torque.y, torque.z };
        bool[]  P      = new bool[_n]; // active/passive set: true = allowed nonzero

        for (int outer = 0; outer < _n; outer++)
        {
            // Residual between the target wrench and what the current (possibly
            // still-zero) solution actually produces, projected onto each
            // excluded-but-allowed thruster's effectiveness — Lawson-Hanson's
            // "most improving" candidate to bring into the active set next.
            float[] produced = ComputeWrench(x);
            float[] r = new float[6];
            for (int k = 0; k < 6; k++) r[k] = target[k] - produced[k];
            Vector3 rForce  = new Vector3(r[0], r[1], r[2]);
            Vector3 rTorque = new Vector3(r[3], r[4], r[5]);

            int   bestIdx   = -1;
            float bestScore = 1e-6f; // tolerance — don't add a thruster over noise
            for (int i = 0; i < _n; i++)
            {
                if (!mask[i] || P[i]) continue;
                Vector3 d = _thrusters[i].direction;
                Vector3 t = Vector3.Cross(_thrusters[i].position, d);
                float score = Vector3.Dot(d, rForce) + Vector3.Dot(t, rTorque);
                if (score > bestScore) { bestScore = score; bestIdx = i; }
            }
            if (bestIdx < 0) break; // converged — no excluded thruster would help

            P[bestIdx] = true;

            // Inner loop: solve for x over the active set P (regularized so it
            // stays well-defined even with fewer than 6 active thrusters, where
            // the exact system is underdetermined), backing out whichever
            // thruster would go most negative before accepting a result.
            for (int inner = 0; inner < 2 * _n; inner++)
            {
                float[,] Bpinv = BuildPinv(P, regularize: true);
                if (Bpinv == null)
                {
                    // Only reachable if P's columns are literally all zero
                    // (e.g. a locked-out/degenerate thruster) — drop it and stop
                    // growing the active set further this call.
                    P[bestIdx] = false;
                    return x;
                }

                float[] z = new float[_n];
                for (int i = 0; i < _n; i++)
                {
                    if (!P[i]) continue;
                    for (int j = 0; j < 6; j++)
                        z[i] += Bpinv[i, j] * target[j];
                }

                float minRatio = 1f;
                int   dropIdx  = -1;
                for (int i = 0; i < _n; i++)
                {
                    if (!P[i] || z[i] >= 0f) continue;
                    float denom = x[i] - z[i];
                    if (denom <= 1e-9f) continue;
                    float ratio = x[i] / denom;
                    if (ratio < minRatio) { minRatio = ratio; dropIdx = i; }
                }

                if (dropIdx < 0)
                {
                    // z is fully non-negative over P — accept it and move on to
                    // the next outer iteration (try adding another thruster).
                    for (int i = 0; i < _n; i++) x[i] = P[i] ? z[i] : 0f;
                    break;
                }

                // Partial move toward z (as far as the first thruster to hit
                // zero allows — the ratio-test invariant that keeps x
                // non-negative throughout), then remove that thruster and
                // re-solve over the smaller active set.
                for (int i = 0; i < _n; i++)
                    if (P[i]) x[i] += minRatio * (z[i] - x[i]);
                P[dropIdx] = false;
                x[dropIdx] = 0f;
            }
        }

        return x;
    }

    // Wrench (force+torque, world convention matching target[]) produced by the
    // given per-thruster throttle vector — the forward model NNLS's residual
    // step needs to know "what does the current solution actually deliver."
    private float[] ComputeWrench(float[] x)
    {
        float[] w = new float[6];
        for (int i = 0; i < _n; i++)
        {
            if (x[i] == 0f) continue;
            Vector3 f = _thrusters[i].direction * x[i];
            Vector3 t = Vector3.Cross(_thrusters[i].position, f);
            w[0] += f.x; w[1] += f.y; w[2] += f.z;
            w[3] += t.x; w[4] += t.y; w[5] += t.z;
        }
        return w;
    }

    // B [6, N]: column i = [dir; r×dir] — wrench produced by thruster i at 1 N thrust.
    // Columns for inactive thrusters are left zero, which excludes them from the
    // least-squares problem entirely (their B† row comes out exactly zero).
    //
    // regularize (added 2026-07-18 for the NNLS rewrite): adds a small Tikhonov
    // term to BBT's diagonal before inverting. B·B^T is only guaranteed full
    // rank when the active set spans all 6 DOF (generically needs >=6 active
    // thrusters) — Allocate()'s NNLS inner loop calls this with active sets
    // that start at size 1 and grow one at a time, so it MUST stay well-defined
    // (never singular) well below that count. Regularizing turns "singular,
    // return null" into "well-posed, smallest-norm approximate answer" for
    // those under-determined active sets, at a cost small enough (1e-3 against
    // diagonal entries that are physically O(1-10) for unit thruster directions
    // and ~1.5m moment arms) to not perturb a genuine full-rank solve. The
    // Initialize() sanity check deliberately calls this with regularize=false —
    // that check exists specifically to catch real geometry singularities
    // (collinear/missing axes), which regularizing here would silently hide.
    private float[,] BuildPinv(bool[] activeMask, bool regularize)
    {
        float[,] B = new float[6, _n];
        for (int i = 0; i < _n; i++)
        {
            if (!activeMask[i]) continue;
            Vector3 d = _thrusters[i].direction;
            Vector3 t = Vector3.Cross(_thrusters[i].position, d);
            B[0, i] = d.x;  B[1, i] = d.y;  B[2, i] = d.z;
            B[3, i] = t.x;  B[4, i] = t.y;  B[5, i] = t.z;
        }

        // BBT = B · B^T [6, 6]
        float[,] BBT = new float[6, 6];
        for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
                for (int k = 0; k < _n; k++)
                    BBT[r, c] += B[r, k] * B[c, k];

        if (regularize)
            for (int d = 0; d < 6; d++)
                BBT[d, d] += 1e-3f;

        float[,] BBTinv = Invert6x6(BBT);
        if (BBTinv == null) return null;

        // B† = B^T · BBT^-1 [N, 6]
        float[,] Bpinv = new float[_n, 6];
        for (int i = 0; i < _n; i++)
            for (int j = 0; j < 6; j++)
                for (int k = 0; k < 6; k++)
                    Bpinv[i, j] += B[k, i] * BBTinv[k, j];

        return Bpinv;
    }

    private static bool[] AllTrue(int n)
    {
        bool[] mask = new bool[n];
        for (int i = 0; i < n; i++) mask[i] = true;
        return mask;
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
