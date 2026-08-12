#!/usr/bin/env -S uv run --script
"""Self-calibrating maze grid fit.

Detects the blue post markers in an overhead maze photo and fits the 180 mm
lattice.  Everything geometric is measured from the detections themselves:

    nadir            least-squares intersection of the post streak axes
    pitch            median nearest-neighbour spacing, then refined by ICP
    orientation      4th-power circular mean of neighbour bond directions
    post height      slope of streak length vs radius  ->  camera height
    truncation cut   upper-envelope quantile fit of that same relation

The only hand-set numbers left are the HSV gate and a minimum blob area.
"""

import cv2
import numpy as np
from scipy.cluster.hierarchy import fcluster, linkage
from scipy.optimize import minimize
from scipy.spatial import KDTree

PITCH_MM = 185.0
POST_H_MM = 50.0  # only used to report camera height; not fitted against


# ---------------------------------------------------------------- detection
def detect_blobs(img_bgr, hsv_lo=(80, 55, 75), hsv_hi=(110, 255, 255), min_area=6):
    """Connected components of the blue mask, as arrays of pixel coords.

    No morphological closing: at ~50 px pitch any kernel big enough to join
    a post's fragments also bridges neighbouring posts.
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lo, hsv_hi)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_area:
            continue
        ys, xs = np.nonzero(lab == i)
        out.append(np.stack([xs, ys], 1).astype(float))
    if len(out) < 8:
        raise RuntimeError(f"only {len(out)} blobs -- widen the HSV gate")
    return out, mask


# -------------------------------------------------------------------- nadir
def _major_axis(P):
    c = P.mean(0)
    w, V = np.linalg.eigh(np.cov((P - c).T))
    if w[0] <= 0:
        return None
    return c, V[:, 1], float(np.sqrt(w[1] / w[0]))


def fit_nadir(blobs, min_aspect=3.0, iters=4):
    """Vertical posts project as lines converging on the nadir.

    Least-squares point of closest approach to all streak axes, with
    iterative reweighting so a few mis-shaped blobs can't drag it.
    """
    axes = [
        a
        for a in (_major_axis(P) for P in blobs)
        if a is not None and a[2] >= min_aspect
    ]
    if len(axes) < 6:
        return None, 0
    w = np.ones(len(axes))
    x = None
    for _ in range(iters):
        A = np.zeros((2, 2))
        b = np.zeros(2)
        for wi, (c, u, _) in zip(w, axes):
            M = np.eye(2) - np.outer(u, u)
            A += wi * M
            b += wi * (M @ c)
        x = np.linalg.solve(A, b)
        d = np.array(
            [np.linalg.norm((np.eye(2) - np.outer(u, u)) @ (x - c)) for c, u, _ in axes]
        )
        s = max(np.median(d), 1e-6)
        w = 1.0 / (1.0 + (d / (3 * s)) ** 2)  # Cauchy weights
    return x, len(axes)


# ------------------------------------------------------- streak length model
def _quantreg(x, y, tau=0.8):
    """Quantile regression -- tracks the upper envelope, i.e. the blobs that
    were not cut short by an adjoining wall."""

    def loss(p):
        r = y - (p[0] * x + p[1])
        return np.sum(np.maximum(tau * r, (tau - 1) * r))

    p0 = np.polyfit(x, y, 1)
    return minimize(
        loss,
        p0,
        method="Nelder-Mead",
        options=dict(xatol=1e-6, fatol=1e-6, maxiter=4000),
    ).x


def nadir_quality(blobs, nadir, min_aspect=3.0):
    """How well does a candidate nadir explain the streak directions?

    Each post's streak should point straight at it.  Returns the median
    angle between streak axis and radial direction, and the median
    perpendicular miss distance -- both are 0 for a perfect nadir.
    Use this to sanity-check a hand-supplied value.
    """
    ang, perp = [], []
    nadir = np.asarray(nadir, float)
    for P in blobs:
        a = _major_axis(P)
        if a is None or a[2] < min_aspect:
            continue
        c, u, _ = a
        v = c - nadir
        nv = np.linalg.norm(v)
        if nv < 1e-6:
            continue
        ang.append(np.degrees(np.arccos(min(1.0, abs((v / nv) @ u)))))
        perp.append(np.linalg.norm((np.eye(2) - np.outer(u, u)) @ (nadir - c)))
    if not ang:
        return dict(n=0, median_angle_deg=np.nan, median_perp_px=np.nan)
    return dict(
        n=len(ang),
        median_angle_deg=float(np.median(ang)),
        median_perp_px=float(np.median(perp)),
    )


def streak_model(blobs, nadir, tau=0.8, keep_frac=0.6):
    """Per-blob radial geometry + the length-vs-radius law.

    A post of height h at ground radius R images with its base at r and its
    top at r*H/(H-h), so streak length is proportional to radius.  Blobs
    falling well under that line have been truncated by a wall.
    """
    rmin, rmax, cen = [], [], []
    for P in blobs:
        r = np.linalg.norm(P - nadir, axis=1)
        rmin.append(r.min())
        rmax.append(r.max())
        cen.append(P.mean(0))
    rmin = np.array(rmin)
    rmax = np.array(rmax)
    cen = np.array(cen)
    L = rmax - rmin
    a, b = _quantreg(rmin, L, tau)
    untruncated = L >= keep_frac * (a * rmin + b)
    return dict(
        rmin=rmin,
        rmax=rmax,
        L=L,
        centroid=cen,
        slope=a,
        intercept=b,
        untruncated=untruncated,
    )


# ------------------------------------------------------------------- lattice
def _rot(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]])


def _umeyama(src, dst, with_scale):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = ((dst - mu_d).T @ (src - mu_s)) / len(src)
    U, D, Vt = np.linalg.svd(S)
    W = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        W[-1, -1] = -1
    R = U @ W @ Vt
    s = (
        float(np.trace(np.diag(D) @ W) / (((src - mu_s) ** 2).sum() / len(src)))
        if with_scale
        else 1.0
    )
    return s, R, mu_d - s * R @ mu_s


def estimate_pitch(pts):
    d, _ = KDTree(pts).query(pts, k=2)
    return float(np.median(d[:, 1]))


def fit_lattice(pts, pitch, max_iter=12):
    """Assign integer indices, refining pitch and pose by ICP against Z^2."""
    tree = KDTree(pts)
    pairs = np.array(list(tree.query_pairs(r=1.2 * pitch)))
    d = pts[pairs[:, 1]] - pts[pairs[:, 0]]
    z = (d[:, 0] + 1j * d[:, 1]) / np.linalg.norm(d, axis=1)
    theta = np.angle(np.sum(z**4)) / 4.0  # 90deg ambiguous

    q = pts @ _rot(theta)
    ph = np.angle(np.sum(np.exp(2j * np.pi * q / pitch), axis=0))
    idx = np.round((q - pitch * ph / (2 * np.pi)) / pitch).astype(int)

    for _ in range(max_iter):
        s, R, t = _umeyama(pitch * idx.astype(float), pts, True)
        new = np.round(((pts - t) @ R / s) / pitch).astype(int)
        if np.array_equal(new, idx):
            break
        idx = new

    if len(np.unique(idx, axis=0)) != len(idx):
        raise RuntimeError("index collisions -- pitch or orientation is wrong")
    idx -= idx.min(0)
    s, R, t = _umeyama(pitch * idx.astype(float), pts, True)
    return idx, float(np.arctan2(R[1, 0], R[0, 0])), s * pitch


# ---------------------------------------------------------------------- main
def solve(
    img_bgr,
    nadir=None,
    pitch_mm=PITCH_MM,
    post_h_mm=POST_H_MM,
    tau=0.8,
    keep_frac=0.6,
    **kw,
):
    """Fit the maze lattice.

    nadir : None to fit it from the streak axes (default), or an (x, y) pixel
            pair to pin it -- e.g. a calibrated principal point.  A supplied
            value is used as-is; check result["nadir_quality"] to see whether
            the streaks actually agree with it.
    """
    blobs, mask = detect_blobs(img_bgr, **kw)

    if nadir is None:
        nadir, n_ax = fit_nadir(blobs)
        nadir_source = "fitted"
        if nadir is None:
            nadir = np.array([img_bgr.shape[1] / 2, img_bgr.shape[0] / 2])
            n_ax, nadir_source = 0, "image centre (fallback)"
    else:
        nadir = np.asarray(nadir, float).reshape(2)
        n_ax, nadir_source = 0, "supplied"

    nq = nadir_quality(blobs, nadir)
    sm = streak_model(blobs, nadir, tau=tau, keep_frac=keep_frac)

    # merge blobs sharing a post; threshold scales off a first pitch guess
    p0 = estimate_pitch(sm["centroid"])
    lab = fcluster(
        linkage(sm["centroid"], method="single"), t=0.3 * p0, criterion="distance"
    )
    posts = np.array([sm["centroid"][lab == k].mean(0) for k in np.unique(lab)])
    clean = np.array([bool(sm["untruncated"][lab == k].all()) for k in np.unique(lab)])

    # fit the lattice on untruncated posts only, then index everything
    pitch_px = estimate_pitch(posts[clean])
    idx_c, theta, pitch_fit = fit_lattice(posts[clean], pitch_px)

    world_c = pitch_mm * idx_c.astype(float)
    H, _ = cv2.findHomography(world_c, posts[clean], method=0)
    Hinv = np.linalg.inv(H)

    all_mm = cv2.perspectiveTransform(posts.reshape(-1, 1, 2), Hinv).reshape(-1, 2)
    idx_all = np.round(all_mm / pitch_mm).astype(int)

    proj = cv2.perspectiveTransform(world_c.reshape(-1, 1, 2), H).reshape(-1, 2)
    rms = float(np.sqrt(((proj - posts[clean]) ** 2).sum(1).mean()))

    # centroids sit at mid post height; rescale about the nadir to reach the floor
    k_top = 1.0 + sm["slope"]
    k_mid = 1.0 + sm["slope"] / 2.0
    floor = nadir + (posts - nadir) / k_mid
    H_floor, _ = cv2.findHomography(
        pitch_mm * idx_all[clean].astype(float), floor[clean], method=0
    )

    return dict(
        blobs=blobs,
        mask=mask,
        nadir=nadir,
        n_axes=n_ax,
        nadir_source=nadir_source,
        nadir_quality=nq,
        posts=posts,
        clean=clean,
        idx=idx_all,
        streak=sm,
        theta=theta,
        pitch_px=pitch_fit,
        mm_per_px=pitch_mm / pitch_fit,
        H_mid=H,
        H_floor=H_floor,
        rms_px=rms,
        cam_height_mm=(
            post_h_mm * k_top / sm["slope"] if sm["slope"] > 1e-6 else np.inf
        ),
    )


if __name__ == "__main__":
    from matplotlib import pyplot as plt

    img = cv2.imread("mazes/1.png")
    assert img is not None, "Image not read"
    # r = solve(img, nadir=(509.0, 287.0))   # <- pin it if you have a value
    r = solve(img)

    mmpx = r["mm_per_px"]
    q = r["nadir_quality"]
    print(f"blobs            {len(r['blobs'])}  ({r['n_axes']} usable streak axes)")
    print(
        f"nadir            ({r['nadir'][0]:.1f}, {r['nadir'][1]:.1f}) px "
        f"[{r['nadir_source']}]"
    )
    print(
        f"  agreement      {q['median_angle_deg']:.1f} deg off radial, "
        f"{q['median_perp_px']:.1f} px miss, over {q['n']} streaks"
    )
    print(f"pitch            {r['pitch_px']:.2f} px   ->  {mmpx:.4f} mm/px")
    print(f"rotation         {np.degrees(r['theta']) % 90:.3f} deg")
    print(
        f"streak slope     {r['streak']['slope']:+.4f}  ->  camera "
        f"{r['cam_height_mm'] / 1000:.2f} m above the deck"
    )
    print(f"posts            {len(r['posts'])} total, {r['clean'].sum()} untruncated")
    print(f"grid             {r['idx'].max(0)[0] + 1} x {r['idx'].max(0)[1] + 1} nodes")
    print(f"fit rms          {r['rms_px']:.2f} px  ({r['rms_px'] * mmpx:.1f} mm)")

    def px_to_mm(p, floor=True):
        Hm = r["H_floor"] if floor else r["H_mid"]
        return cv2.perspectiveTransform(
            np.asarray(p, float).reshape(-1, 1, 2), np.linalg.inv(Hm)
        ).reshape(-1, 2)

    fig, ax = plt.subplots(1, 2, figsize=(13, 6))
    ax[0].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    w = 180.0 * r["idx"].astype(float)
    pr = cv2.perspectiveTransform(w.reshape(-1, 1, 2), r["H_mid"]).reshape(-1, 2)
    ax[0].plot(pr[r["clean"], 0], pr[r["clean"], 1], "o", mfc="none", mec="lime", ms=9)
    ax[0].plot(
        pr[~r["clean"], 0], pr[~r["clean"], 1], "s", mfc="none", mec="orange", ms=9
    )
    ax[0].plot(*r["nadir"], "c+", ms=14, mew=2)
    ax[0].set_title("lattice: clean (green), truncated (orange), nadir (cyan)")
    ax[0].set_xlim(300, 840)
    ax[0].set_ylim(560, 20)

    s = r["streak"]
    ax[1].scatter(
        s["rmin"][s["untruncated"]], s["L"][s["untruncated"]], s=18, label="untruncated"
    )
    ax[1].scatter(
        s["rmin"][~s["untruncated"]],
        s["L"][~s["untruncated"]],
        s=18,
        c="orange",
        label="truncated",
    )
    xs = np.linspace(0, s["rmin"].max(), 2)
    ax[1].plot(xs, s["slope"] * xs + s["intercept"], "k-", lw=1)
    ax[1].plot(xs, 0.6 * (s["slope"] * xs + s["intercept"]), "k--", lw=1)
    ax[1].set_xlabel("base radius from nadir (px)")
    ax[1].set_ylabel("streak length (px)")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    ax[1].set_title("post height model")
    plt.tight_layout()
    plt.savefig("fit.png", dpi=95)
    plt.show()
