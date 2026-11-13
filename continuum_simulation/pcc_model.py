"""
Piecewise Constant-Curvature (PCC) model of a tendon-driven continuum robot
inside a parametric sink drain / P-trap.

First-pass (Tier 1a) model for the MREN 403 drain-cleaning robot. It answers:
  1. Can a given design (length, number of sections, disks, spring) follow
     the P-trap without hitting the walls, at each insertion depth?
  2. What tendon tension does each section need to hold that shape,
     including capstan friction through the bends?
  3. How do tendon offset and spring wire diameter trade off?

Units: mm, N, rad unless noted.  EI is in N*mm^2.

Usage
-----
    python pcc_model.py                                   # baseline values below
    python pcc_model.py --params continuum_sim_parameters.xlsx   # read baselines from workbook
    python pcc_model.py --no-gif                          # skip the animation

Model assumptions (state these in the report)
---------------------------------------------
* Each section bends with constant curvature (Webster & Jones 2010 PCC model).
* Planar bending in the plane of the trap for the fit; 3-D kinematics and
  tendon-length mapping are included for later use.
* The KUKA holds the robot base pointing straight down above the outlet and
  inserts along the drain axis, so the base always lies on the vertical line.
* "Conformance error" = max distance from robot backbone to drain centreline
  with the robot in free space (no wall contact). Exceeding the radial
  clearance (ID - OD)/2 means the robot would touch the wall.
* Spring EI from Wahl's helical-spring bending formula; fluid line ignored.
* Tendon tension from moment balance M = T*r with tendons for section j
  passing through sections 1..j; friction via capstan e^(mu*theta).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from scipy.spatial import cKDTree


# --------------------------------------------------------------------------
# Parameters 
# --------------------------------------------------------------------------
@dataclass
class Design:
    L: float = 500.0        # total continuum length
    N_sec: int = 2          # number of sections
    E: float = 193.0        # GPa, spring wire
    G: float = 69.0         # GPa
    d: float = 0.8          # spring wire diameter
    D: float = 6.0          # mean coil diameter
    p: float = 1.2          # pitch
    s: float = 10.0         # disk spacing
    OD: float = 20.0        # disk outer diameter
    t: float = 2.0          # disk thickness
    r_t: float = 7.0        # tendon offset radius
    mu: float = 0.15        # tendon-disk friction
    T_max: float = 30.0     # max actuator tension, N
    section_lengths: list[float] | None = field(default_factory=lambda: [200, 126, 30, 63, 81])  # optional custom split

    @property
    def lengths(self) -> np.ndarray:
        if self.section_lengths:
            return np.asarray(self.section_lengths, float)
        return np.full(self.N_sec, self.L / self.N_sec)

    @property
    def EI(self) -> float:
        """Effective bending stiffness of the coil spring [N*mm^2] (Wahl)."""
        E = self.E * 1e3  # GPa -> N/mm^2
        G = self.G * 1e3
        return E * self.d**4 * self.p / (32 * self.D * (1 + E / (2 * G)))

    @property
    def R_min(self) -> float:
        """Geometric minimum bend radius: disk-edge contact or coil bind."""
        r_disk = (self.OD / 2) / (1 - self.t / self.s)
        r_coil = (self.D / 2) / (1 - self.d / self.p) if self.p > self.d else 0.0
        return max(r_disk, r_coil)


@dataclass
class Drain:
    ID: float = 38.1        # inner diameter: 38.1mm (1.5"), 31.75mm (1 1/4"), 50.8mm (2")
    H_out: float = 800.0    # sink outlet height above floor
    L_tail: float = 200.0   # tailpiece length
    R_trap: float = 40.0    # U-bend centreline radius (also used for the 90 deg bend)
    h_seal: float = 75.0    # water seal depth
    L_arm: float = 300.0    # horizontal trap arm


# --------------------------------------------------------------------------
# Optional: read baselines from the parameter workbook
# --------------------------------------------------------------------------
def load_params(xlsx: Path) -> tuple[Design, Drain]:
    from openpyxl import load_workbook
    wb = load_workbook(xlsx, data_only=True)
    vals = {}
    for sheet in ("Design Inputs", "Drain Geometry"):
        for row in wb[sheet].iter_rows(min_row=2, values_only=True):
            sym, base = row[2], row[4]
            if isinstance(base, (int, float)):
                vals[sym] = float(base)
    des = Design(**{k: (int(v) if k == "N_sec" else v)
                    for k, v in vals.items() if k in Design.__dataclass_fields__})
    drn = Drain(**{k: v for k, v in vals.items() if k in Drain.__dataclass_fields__})
    return des, drn


# --------------------------------------------------------------------------
# Planar constant-curvature integration (used for BOTH robot and drain path)
# --------------------------------------------------------------------------
def arc_points(p0, th0, kappa, length, n):
    """Points along a constant-curvature arc starting at p0 with heading th0."""
    u = np.linspace(0.0, length, n)
    if abs(kappa) < 1e-9:
        pts = p0 + np.outer(u, [math.cos(th0), math.sin(th0)])
    else:
        th = th0 + kappa * u
        pts = p0 + np.column_stack([(np.sin(th) - math.sin(th0)) / kappa,
                                    (-np.cos(th) + math.cos(th0)) / kappa])
    return pts, th0 + kappa * length


def chain(p0, th0, kappas, lengths, pts_per_mm=0.5):
    """Integrate a chain of constant-curvature segments. Returns points,
    headings and the index where each segment ends."""
    pts, heads, ends = [np.asarray(p0, float)], [th0], []
    p, th = np.asarray(p0, float), th0
    for k, l in zip(kappas, lengths):
        n = max(int(l * pts_per_mm), 3)
        seg, th_end = arc_points(p, th, k, l, n)
        u = np.linspace(0, l, n)
        pts.extend(seg[1:]); heads.extend(th + k * u[1:])
        p, th = seg[-1], th_end
        ends.append(len(pts) - 1)
    return np.array(pts), np.array(heads), ends


# --------------------------------------------------------------------------
# Drain / P-trap geometry
# --------------------------------------------------------------------------
@dataclass
class DrainPath:
    drain: Drain
    ext: float                       # straight extension above the outlet
    segs: list = field(default_factory=list)  # (name, kappa, length)

    def __post_init__(self):
        dr = self.drain
        R = dr.R_trap
        # Rising leg so that the water seal depth comes out right:
        # weir invert - top of dip = h_seal  ->  rise = h_seal - 2R + ID
        rise = max(dr.h_seal - 2 * R + dr.ID, 0.0)
        self.segs = [
            ("above outlet", 0.0, self.ext),
            ("tailpiece", 0.0, dr.L_tail),
            ("U-bend", +1 / R, math.pi * R),
            ("rising leg", 0.0, rise),
            ("90° bend", -1 / R, math.pi * R / 2),
            ("trap arm", 0.0, dr.L_arm),
        ]
        start = np.array([0.0, dr.H_out + self.ext])
        k = [s[1] for s in self.segs]; l = [s[2] for s in self.segs]
        self.pts, self.heads, ends = chain(start, -math.pi / 2, k, l, pts_per_mm=2)
        # arc-length coordinate with s = 0 at the sink outlet
        steps = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
        self.s = np.concatenate([[0], np.cumsum(steps)]) - self.ext
        self.tree = cKDTree(self.pts)
        self.feature_s = np.cumsum([0] + l)[1:] - self.ext  # end of each segment

    def at(self, s):
        """Position and heading on the centreline at arc length s."""
        i = int(np.clip(np.searchsorted(self.s, s), 0, len(self.s) - 1))
        return self.pts[i], self.heads[i]

    def interp(self, s):
        """Centreline points at arc lengths s (array)."""
        return np.column_stack([np.interp(s, self.s, self.pts[:, 0]),
                                np.interp(s, self.s, self.pts[:, 1])])

    def mean_kappa(self, s0, s1):
        """Average path curvature over [s0, s1] (used as an initial guess)."""
        acc, edges = 0.0, np.concatenate([[-self.ext], self.feature_s])
        for (name, k, l), a, b in zip(self.segs, edges[:-1], edges[1:]):
            acc += k * max(0.0, min(b, s1) - max(a, s0))
        return acc / (s1 - s0)

    def walls(self):
        nrm = np.column_stack([-np.sin(self.heads), np.cos(self.heads)])
        h = self.drain.ID / 2
        return self.pts + h * nrm, self.pts - h * nrm


# --------------------------------------------------------------------------
# Fit the robot to the drain at a given insertion depth
# --------------------------------------------------------------------------
@dataclass
class Fit:
    depth: float
    kappas: np.ndarray
    pts: np.ndarray
    ends: list
    max_err: float
    feasible: bool
    T_act: np.ndarray  # actuator-side tension per section, N


def fit_robot(des: Design, path: DrainPath, depth: float) -> Fit:
    """Best-fit section curvatures so the backbone follows the centreline
    with the tip at arc length `depth` past the outlet."""
    lens = des.lengths
    s_base = depth - lens.sum()
    p0, th0 = path.at(s_base)
    k_max = 1 / des.R_min

    # Initial guess: average path curvature under each section
    edges = s_base + np.concatenate([[0], np.cumsum(lens)])
    k0 = np.array([path.mean_kappa(a, b) for a, b in zip(edges[:-1], edges[1:])])
    k0 = np.clip(k0, -k_max, k_max)

    # Follow-the-leader cost: robot point at arc u should sit on the
    # centreline point at arc s_base + u (avoids "curled-up" false fits).
    pts0, _, _ = chain(p0, th0, k0, lens)
    u = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts0, axis=0), axis=1))])
    target = path.interp(s_base + u)

    def cost(k):
        pts, _, _ = chain(p0, th0, k, lens)
        dist = np.linalg.norm(pts - target, axis=1)
        return np.mean(dist**2) + 0.05 * np.max(dist) ** 2

    res = minimize(cost, k0, method="L-BFGS-B", bounds=[(-k_max, k_max)] * len(lens))
    k = res.x
    pts, _, ends = chain(p0, th0, k, lens)
    max_err = float(np.max(path.tree.query(pts)[0]))
    clearance = (path.drain.ID - des.OD) / 2

    # Tendon tensions: moment in section j = EI*k_j = r * (net tension of
    # tendons ending at sections >= j)  ->  net tension for section j:
    k_next = np.append(k[1:], 0.0)
    T_tip = des.EI * np.abs(k - k_next) / des.r_t
    # Capstan: tendon for section j wraps through sections 1..j
    wrap = np.cumsum(np.abs(k) * lens)
    T_act = T_tip * np.exp(des.mu * wrap)
    return Fit(depth, k, pts, ends, max_err, max_err <= clearance, T_act)


# --------------------------------------------------------------------------
# 3-D PCC kinematics + tendon mapping (for the later Cosserat / MuJoCo work)
# --------------------------------------------------------------------------
def pcc_transform(kappa, phi, l):
    """4x4 homogeneous transform of one constant-curvature section
    (Webster & Jones 2010). phi = bending-plane angle."""
    c, s = math.cos(phi), math.sin(phi)
    if abs(kappa) < 1e-9:
        R = np.eye(3); p = np.array([0, 0, l])
    else:
        th = kappa * l
        ct, st = math.cos(th), math.sin(th)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        Ry = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
        R = Rz @ Ry @ Rz.T
        p = np.array([c * (1 - ct), s * (1 - ct), st]) / kappa
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    return T


def tendon_displacements(kappas, phis, lengths, r_t, n_t=3, offset=0.0):
    """Length change of every tendon (rows = sections the tendons end at,
    cols = tendon index). Negative = tendon must be pulled in."""
    sig = offset + 2 * np.pi * np.arange(n_t) / n_t
    dl = np.zeros((len(kappas), n_t))
    for j in range(len(kappas)):
        for k in range(j + 1):  # tendon for section j passes sections 0..j
            dl[j] += -lengths[k] * kappas[k] * r_t * np.cos(phis[k] - sig)
    return dl


# --------------------------------------------------------------------------
# Plotting
# --------------------------------------------------------------------------
COLS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b", "#17becf"]


def draw_drain(ax, path: DrainPath):
    a, b = path.walls()
    show = path.s >= -60
    ax.plot(a[show, 0], a[show, 1], "k", lw=1.5)
    ax.plot(b[show, 0], b[show, 1], "k", lw=1.5)
    ax.plot(path.pts[show, 0], path.pts[show, 1], "k:", lw=0.7)
    ax.axhline(path.drain.H_out, color="0.6", lw=0.8, ls="--")
    ax.set_aspect("equal"); ax.set_xlabel("x [mm]"); ax.set_ylabel("height [mm]")


def draw_robot(ax, des: Design, fit: Fit):
    start = 0
    for j, e in enumerate(fit.ends):
        seg = fit.pts[start:e + 1]
        ax.plot(seg[:, 0], seg[:, 1], color=COLS[j % len(COLS)], lw=2.5,
                label=f"section {j+1}")
        start = e
    # spacer disks
    pts = fit.pts
    arclen = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    for sd in np.arange(des.s, arclen[-1], des.s):
        i = np.searchsorted(arclen, sd)
        i = min(max(i, 1), len(pts) - 1)
        tang = pts[i] - pts[i - 1]; tang /= np.linalg.norm(tang)
        n = np.array([-tang[1], tang[0]]) * des.OD / 2
        ax.plot(*np.array([pts[i] - n, pts[i] + n]).T, color="0.3", lw=0.6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", type=Path, help="parameter workbook (.xlsx)")
    ap.add_argument("--out", type=Path, default=Path(__file__).parent / "figures")
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()

    des, drn = load_params(args.params) if args.params else (Design(), Drain())
    args.out.mkdir(parents=True, exist_ok=True)
    path = DrainPath(drn, ext=des.L + 50)
    clearance = (drn.ID - des.OD) / 2
    depths = np.linspace(20, des.L, 60)

    print(f"EI = {des.EI:.1f} N·mm²   R_min = {des.R_min:.1f} mm   "
          f"radial clearance = {clearance:.1f} mm")
    print("Drain features (arc length past outlet, mm):",
          {n: round(s, 1) for (n, _, _), s in zip(path.segs[1:], path.feature_s[1:])})

    # ---- Fig 1: snapshots of best-fit shape (baseline design, 3 sections) --
    d3 = replace(des, N_sec=3)
    snaps = [0.35, 0.6, 0.8, 1.0]
    fig, axs = plt.subplots(1, 4, figsize=(16, 6), sharey=True)
    for ax, f in zip(axs, snaps):
        fit = fit_robot(d3, path, f * des.L)
        draw_drain(ax, path); draw_robot(ax, d3, fit)
        ax.set_title(f"tip {fit.depth:.0f} mm past outlet\n"
                     f"max error {fit.max_err:.1f} mm "
                     f"({'OK' if fit.feasible else 'hits wall'})", fontsize=10)
        ax.set_xlim(-60, 3 * drn.R_trap + 220)
        ax.set_ylim(drn.H_out - drn.L_tail - drn.R_trap - 60, drn.H_out + 80)
    axs[0].legend(loc="upper right", fontsize=8)
    fig.suptitle(f"Best-fit PCC shape through P-trap — L={d3.L:.0f} mm, 3 equal sections, "
                 f"ID={drn.ID:.0f} mm, R_trap={drn.R_trap:.0f} mm")
    fig.tight_layout(); fig.savefig(args.out / "1_snapshots.png", dpi=150); plt.close(fig)

    # ---- Fig 2: conformance error vs depth for N_sec = 1..4 ---------------
    fig, ax = plt.subplots(figsize=(9, 5))
    results = {}
    for i, n in enumerate([1, 2, 3, 4, 6, 8]):
        dn = replace(des, N_sec=n)
        fits = [fit_robot(dn, path, d) for d in depths]
        results[n] = fits
        ax.plot(depths, [f.max_err for f in fits], color=COLS[i], lw=2,
                label=f"{n} section{'s' if n > 1 else ''}")
    ax.axhline(clearance, color="k", ls="--", label=f"radial clearance {clearance:.1f} mm")
    ax.set_ylim(0, 60)  # 1-section curve runs off the top; keep the rest readable
    for (name, _, _), s in zip(path.segs[1:-1], path.feature_s[1:-1]):
        if s < des.L:
            ax.axvline(s, color="0.7", lw=0.8)
            ax.text(s, ax.get_ylim()[1] * 0.95, f" end of {name}", rotation=90,
                    va="top", fontsize=8, color="0.4")
    ax.set_xlabel("tip insertion depth past outlet [mm]")
    ax.set_ylabel("max backbone deviation from centreline [mm]")
    ax.set_title("How many sections does it take to follow the trap in free space?\n(equal-length sections, no wall contact)")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(args.out / "2_sections_vs_conformance.png", dpi=150); plt.close(fig)

    # ---- Fig 3: actuator tension per section vs depth (3 sections) --------
    fig, ax = plt.subplots(figsize=(9, 5))
    T = np.array([f.T_act for f in results[3]])
    for j in range(3):
        ax.plot(depths, T[:, j], color=COLS[j], lw=2, label=f"section {j+1} tendon")
    ax.axhline(des.T_max, color="k", ls="--", label=f"T_max = {des.T_max:.0f} N")
    ax.set_xlabel("tip insertion depth past outlet [mm]")
    ax.set_ylabel("actuator-side tendon tension [N]")
    ax.set_title(f"Tendon tension incl. capstan friction (μ={des.mu}), 3 sections, "
                 f"EI={des.EI:.0f} N·mm²")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(args.out / "3_tendon_tension.png", dpi=150); plt.close(fig)

    # ---- Fig 4: design chart — tension to hold trap radius ---------------
    fig, ax = plt.subplots(figsize=(9, 5))
    dw = np.linspace(0.4, 1.4, 50)
    wrap = math.pi * 1.5  # U-bend + 90 deg bend
    for i, r in enumerate([4, 6, 8, 10]):
        Tn = [replace(des, d=x).EI / (drn.R_trap * r) * math.exp(des.mu * wrap) for x in dw]
        ax.plot(dw, Tn, color=COLS[i], lw=2, label=f"tendon offset r_t = {r} mm")
    ax.axhline(des.T_max, color="k", ls="--", label=f"T_max = {des.T_max:.0f} N")
    ax.axvline(des.d, color="0.6", ls=":", label=f"baseline d = {des.d} mm")
    ax.set_yscale("log")
    ax.set_xlabel("spring wire diameter d [mm]")
    ax.set_ylabel("actuator tension to bend to R_trap [N]")
    ax.set_title(f"Design chart: D={des.D} mm, p={des.p} mm, μ={des.mu}, R_trap={drn.R_trap:.0f} mm")
    ax.legend(); ax.grid(alpha=0.3, which="both")
    fig.tight_layout(); fig.savefig(args.out / "4_design_chart.png", dpi=150); plt.close(fig)

    # ---- Summary table ------------------------------------------------------
    print("\nN_sec | deepest feasible tip depth [mm] | peak tension (3-sec only)")
    for n, fits in results.items():
        ok = [f.depth for f in fits if f.feasible]
        # deepest depth such that every shallower depth was also feasible
        reach = 0.0
        for f in fits:
            if not f.feasible: break
            reach = f.depth
        extra = f"  {T.max():.2f} N" if n == 3 else ""
        print(f"  {n}   | {reach:6.0f}{extra}")

    # ---- 3-D sanity check of kinematics + tendon mapping ------------------
    k3, ph3, l3 = [0.01, 0.02], [0.0, math.pi / 2], [100.0, 100.0]
    Ttip = pcc_transform(k3[0], ph3[0], l3[0]) @ pcc_transform(k3[1], ph3[1], l3[1])
    print("\n3-D check: tip position for 2 sections (κ=0.01,0.02 /mm; φ=0,90°):",
          np.round(Ttip[:3, 3], 1))
    print("tendon Δl [mm] (rows: section, cols: tendon 0°,120°,240°):\n",
          np.round(tendon_displacements(k3, ph3, l3, des.r_t), 2))

    # ---- Optional animation -------------------------------------------------
    if not args.no_gif:
        from matplotlib.animation import FuncAnimation, PillowWriter
        fig, ax = plt.subplots(figsize=(6, 6))
        frames = results[3]
        def update(i):
            ax.clear(); draw_drain(ax, path); draw_robot(ax, d3, frames[i])
            ax.set_xlim(-60, 3 * drn.R_trap + 220)
            ax.set_ylim(drn.H_out - drn.L_tail - drn.R_trap - 60, drn.H_out + 80)
            f = frames[i]
            ax.set_title(f"depth {f.depth:.0f} mm — max error {f.max_err:.1f} mm "
                         f"({'OK' if f.feasible else 'hits wall'})", fontsize=10)
        anim = FuncAnimation(fig, update, frames=len(frames))
        anim.save(args.out / "insertion.gif", writer=PillowWriter(fps=8))
        plt.close(fig)

    print(f"\nFigures written to {args.out.resolve()}")


if __name__ == "__main__":
    main()
