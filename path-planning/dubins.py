#!/usr/bin/env -S uv run --script
"""Dubins curves -- the shortest bounded-curvature path between two poses.

A Dubins path is three primitives drawn from {L, S, R} -- a minimum-radius left
turn, a straight, a minimum-radius right turn -- and one of six words (LSL, LSR,
RSL, RSR, RLR, LRL) is always the optimum.  That alphabet is exactly what the
firmware's ``Segment`` understands, which is the whole reason the planner steers
with these instead of straight RRT edges: the curve that gets collision checked
is the curve that gets driven, with no smoothing pass afterwards to invalidate
the check.

Everything is vectorised over a leading axis so RRT* can score an entire tree
against one sample in a single call -- the tree query is the hot loop, and six
closed forms over an (n,) array is two orders of magnitude cheaper than a Python
loop over nodes.  Scalar entry points are thin wrappers.

Frames
------
Angles are radians, CCW positive, in whatever right-handed frame the caller
works in.  ``L`` therefore means "turn CCW".  The maze map is in image
convention (+y down), so a map-frame ``L`` is a physical right turn; the flip
into robot coordinates lives in ``segments.to_firmware_frame`` rather than here.
"""

import numpy as np

TWO_PI = 2.0 * np.pi

# Word order is fixed: everything downstream indexes into it.
WORDS = ("LSL", "LSR", "RSL", "RSR", "RLR", "LRL")
MODES = {w: tuple(w) for w in WORDS}


def mod2pi(x):
    """Wrap to [0, 2pi).  Dubins turn angles are all non-negative by convention."""
    return np.mod(x, TWO_PI)


def wrap_pi(x):
    """Wrap to (-pi, pi]."""
    return -(np.mod(-np.asarray(x, float) + np.pi, TWO_PI) - np.pi)


# ------------------------------------------------------------------- the words
def _all_words(alpha, beta, d):
    """Closed-form (t, p, q) for all six words.

    Standard Shkel & Lumelsky normalisation: the pose pair has been rotated so
    the displacement lies along +x, scaled so the turn radius is 1.  ``t`` and
    ``q`` are turn angles, ``p`` is a straight length for CSC words and the
    middle turn angle for CCC words -- so the normalised path length is t+p+q
    either way.

    Returns ``params`` (6, n, 3) and ``ok`` (6, n).  Infeasible words (a CSC
    word whose circles overlap, a CCC word for poses too far apart) come back
    ``ok=False`` with NaN params.
    """
    alpha = np.atleast_1d(np.asarray(alpha, float))
    beta = np.atleast_1d(np.asarray(beta, float))
    d = np.atleast_1d(np.asarray(d, float))
    n = np.broadcast(alpha, beta, d).shape

    sa, ca = np.sin(alpha), np.cos(alpha)
    sb, cb = np.sin(beta), np.cos(beta)
    c_ab = np.cos(alpha - beta)
    dd = d * d

    params = np.full((6,) + n + (3,), np.nan)
    ok = np.zeros((6,) + n, bool)

    def _set(k, t, p, q, good):
        params[k, ..., 0] = np.where(good, mod2pi(t), np.nan)
        params[k, ..., 1] = np.where(good, p, np.nan)
        params[k, ..., 2] = np.where(good, mod2pi(q), np.nan)
        ok[k] = good

    with np.errstate(invalid="ignore", divide="ignore"):
        # LSL
        psq = 2.0 + dd - 2.0 * c_ab + 2.0 * d * (sa - sb)
        good = psq >= 0.0
        tmp = np.arctan2(cb - ca, d + sa - sb)
        _set(0, tmp - alpha, np.sqrt(np.maximum(psq, 0.0)), beta - tmp, good)

        # LSR
        psq = -2.0 + dd + 2.0 * c_ab + 2.0 * d * (sa + sb)
        good = psq >= 0.0
        p = np.sqrt(np.maximum(psq, 0.0))
        tmp = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
        _set(1, tmp - alpha, p, tmp - beta, good)

        # RSL
        psq = -2.0 + dd + 2.0 * c_ab - 2.0 * d * (sa + sb)
        good = psq >= 0.0
        p = np.sqrt(np.maximum(psq, 0.0))
        tmp = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
        _set(2, alpha - tmp, p, beta - tmp, good)

        # RSR
        psq = 2.0 + dd - 2.0 * c_ab + 2.0 * d * (sb - sa)
        good = psq >= 0.0
        tmp = np.arctan2(ca - cb, d - sa + sb)
        _set(3, alpha - tmp, np.sqrt(np.maximum(psq, 0.0)), tmp - beta, good)

        # RLR
        arg = (6.0 - dd + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
        good = np.abs(arg) <= 1.0
        p = mod2pi(TWO_PI - np.arccos(np.clip(arg, -1.0, 1.0)))
        t = mod2pi(alpha - np.arctan2(ca - cb, d - sa + sb) + p / 2.0)
        _set(4, t, p, alpha - beta - t + p, good)

        # LRL
        arg = (6.0 - dd + 2.0 * c_ab + 2.0 * d * (sb - sa)) / 8.0
        good = np.abs(arg) <= 1.0
        p = mod2pi(TWO_PI - np.arccos(np.clip(arg, -1.0, 1.0)))
        t = mod2pi(-alpha + np.arctan2(-ca + cb, d + sa - sb) + p / 2.0)
        _set(5, t, p, mod2pi(beta) - alpha - t + p, good)

    return params, ok


def _normalise(q0, q1, rho):
    """Pose pair -> (alpha, beta, d) in the canonical Dubins frame."""
    q0 = np.atleast_2d(np.asarray(q0, float))
    q1 = np.atleast_2d(np.asarray(q1, float))
    delta = q1[..., :2] - q0[..., :2]
    D = np.hypot(delta[..., 0], delta[..., 1])
    th = np.arctan2(delta[..., 1], delta[..., 0])
    return mod2pi(q0[..., 2] - th), mod2pi(q1[..., 2] - th), D / rho


# ------------------------------------------------------------------- interface
def all_lengths(q0, q1, rho):
    """Length of every word, ``inf`` where infeasible.  Shape (6, n)."""
    alpha, beta, d = _normalise(q0, q1, rho)
    params, ok = _all_words(alpha, beta, d)
    L = params.sum(-1) * rho
    return np.where(ok, L, np.inf), params


def lengths(q0, q1, rho):
    """Shortest-path length and word index for broadcast pose pairs.

    ``q0`` and ``q1`` broadcast, so this serves both RRT* queries: many tree
    nodes against one sample (nearest / choose-parent) and one new node against
    many tree nodes (rewire).
    """
    L, params = all_lengths(q0, q1, rho)
    L = np.where(np.isnan(L), np.inf, L)
    k = np.argmin(L, axis=0)
    i = np.arange(L.shape[1])
    return L[k, i], k, params[k, i]


def shortest(q0, q1, rho):
    """Scalar version: returns ``(length, word, params)``.

    ``length`` is ``inf`` and ``word`` is ``None`` when no word is feasible,
    which in practice only happens on degenerate input (rho <= 0).
    """
    L, k, p = lengths(np.asarray(q0, float)[None], np.asarray(q1, float)[None], rho)
    if not np.isfinite(L[0]):
        return np.inf, None, None
    return float(L[0]), WORDS[int(k[0])], p[0].copy()


# ----------------------------------------------------------------- propagation
def _advance(q, mode, param, rho):
    """Apply one primitive to a pose.  ``param`` is an angle for L/R, mm for S."""
    x, y, th = q
    if mode == "S":
        return np.array([x + param * np.cos(th), y + param * np.sin(th), th])
    s = 1.0 if mode == "L" else -1.0
    th2 = th + s * param
    return np.array(
        [
            x + s * rho * (np.sin(th2) - np.sin(th)),
            y - s * rho * (np.cos(th2) - np.cos(th)),
            th2,
        ]
    )


def primitives(q0, word, params, rho):
    """Expand a word into three geometric primitives, in world units.

    Each entry is ``(mode, q_start, q_end, value, centre)`` where ``value`` is
    the swept angle (rad, always >= 0) for L/R and the length (mm) for S, and
    ``centre`` is the turn centre (None for a straight).  Zero-length
    primitives are kept -- callers that care drop them, and keeping them here
    makes the reconstruction test trivial.
    """
    q = np.asarray(q0, float).copy()
    out = []
    for mode, v in zip(MODES[word], np.asarray(params, float)):
        step = v * rho if mode == "S" else v
        q_next = _advance(q, mode, v if mode != "S" else step, rho)
        centre = None
        if mode != "S":
            s = 1.0 if mode == "L" else -1.0
            centre = np.array(
                [
                    q[0] - s * rho * np.sin(q[2]),
                    q[1] + s * rho * np.cos(q[2]),
                ]
            )
        out.append((mode, q.copy(), q_next.copy(), float(step), centre))
        q = q_next
    return out


def endpoint(q0, word, params, rho):
    """Pose reached by driving the word from ``q0``."""
    q = np.asarray(q0, float).copy()
    for mode, v in zip(MODES[word], np.asarray(params, float)):
        q = _advance(q, mode, v * rho if mode == "S" else v, rho)
    return q


def sample_poses(q0, word, params, rho, ds, offset=0.0):
    """Poses ``(x, y, theta)`` along the path, endpoints included.

    Used for collision checking, so the guarantee that matters is the spacing
    ceiling.  ``offset`` says the caller is really checking a point that rides
    that many mm ahead of the reference point -- an off-centre robot body -- and
    the ceiling is then honoured for *that* point, which is the one being
    checked.  It is the tighter requirement: around a turn the offset point
    orbits the same centre at ``hypot(rho, offset)``, so it covers that much
    more arc for the same swept angle, and sampling at ``ds/rho`` would leave it
    up to 30% further apart than asked at rho=30, offset=25.
    """
    out = [np.asarray(q0, float)[:3]]
    for mode, qa, qb, value, centre in primitives(q0, word, params, rho):
        if mode == "S":
            if value <= 1e-9:
                continue
            n = max(1, int(np.ceil(value / ds)))
            t = np.arange(1, n + 1)[:, None] / n
            P = qa[:2] + t * (qb[:2] - qa[:2])
            out.append(np.column_stack([P, np.full(n, qa[2])]))
        else:
            if value <= 1e-9:
                continue
            # arc-length spacing, not chord spacing: the collision checker's
            # soundness argument is stated in arc length (MazeWorld.curve_valid)
            dphi = ds / np.hypot(rho, offset)
            n = max(1, int(np.ceil(value / max(dphi, 1e-9))))
            s = 1.0 if mode == "L" else -1.0
            a0 = np.arctan2(qa[1] - centre[1], qa[0] - centre[0])
            a = a0 + s * value * np.arange(1, n + 1) / n
            P = centre + rho * np.stack([np.cos(a), np.sin(a)], 1)
            # the tangent leads the radius by a quarter turn, in the turn's sense
            out.append(np.column_stack([P, a + s * 0.5 * np.pi]))
    return np.concatenate([np.atleast_2d(p) for p in out], 0)


def sample(q0, word, params, rho, ds):
    """Points along the path at spacing <= ``ds``, endpoints included."""
    return sample_poses(q0, word, params, rho, ds)[:, :2]


def truncate(word, params, rho, s):
    """Params for the first ``s`` of arc length of a path.

    A prefix of a Dubins word is still a Dubins word -- the later primitives
    just shrink to zero.  RRT* steering needs this: the edge it validates has to
    be the curve it actually walked, and the shortest path between the near node
    and a *nearby* truncated endpoint is frequently a full 2*pi*rho loop rather
    than the prefix, which no corridor will fit.
    """
    out = np.zeros(3)
    left = float(s)
    for i, (mode, v) in enumerate(zip(MODES[word], np.asarray(params, float))):
        seg = v * rho
        if seg <= 1e-12 or left <= 0.0:
            continue
        take = min(left, seg)
        out[i] = v * (take / seg)
        left -= take
    return out


def interpolate(q0, word, params, rho, s):
    """Pose at arc length ``s`` along the path (clamped to the path)."""
    q = np.asarray(q0, float).copy()
    left = float(s)
    for mode, v in zip(MODES[word], np.asarray(params, float)):
        seg = v * rho if mode == "S" else v * rho
        if left <= 0.0:
            break
        take = min(left, seg)
        frac = 0.0 if seg <= 1e-12 else take / seg
        q = _advance(q, mode, (v * frac) * rho if mode == "S" else v * frac, rho)
        left -= take
    return q
