"""A3-A6 geometry: component identification, wing measurement, section cutting, airfoil reconstruction.

Assumptions (reported, never hidden): the model is axis-aligned in CAD space (wing span along a
principal axis), +thickness axis is "up". Rotated models are reported as ambiguous, not guessed.
"""
import math
import numpy as np
from OCP.TopAbs import TopAbs_SOLID, TopAbs_EDGE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.gp import gp_Pln, gp_Pnt, gp_Dir

AX = "XYZ"


class StageError(Exception):
    def __init__(self, code, message, details=None):
        super().__init__(message)
        self.code, self.message, self.details = code, message, details or {}


def list_solids(shape):
    out = []
    e = TopExp_Explorer(shape, TopAbs_SOLID)
    while e.More():
        out.append(TopoDS.Solid(e.Current()))
        e.Next()
    return out or [shape]  # no solids (surface model): treat whole shape as one candidate


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    lo, hi = b.CornerMin(), b.CornerMax()
    return np.array([lo.X(), lo.Y(), lo.Z()]), np.array([hi.X(), hi.Y(), hi.Z()])


def volume(shape):
    p = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, p)
    return abs(p.Mass())


def _are_split_halves(a, b, tol_frac=0.02):
    """Two wing-like solids that are the two sides of one wing: same axes, same chord/thickness extents,
    and touching end to end along the span axis."""
    if (a["span_axis"], a["chord_axis"], a["thin_axis"]) != (b["span_axis"], b["chord_axis"], b["thin_axis"]):
        return False
    sa, ca, ta = ("XYZ".index(a[k]) for k in ("span_axis", "chord_axis", "thin_axis"))
    amin, amax, bmin, bmax = (np.array(a["bbox_min"]), np.array(a["bbox_max"]),
                              np.array(b["bbox_min"]), np.array(b["bbox_max"]))
    chord = amax[ca] - amin[ca]
    thick = amax[ta] - amin[ta]
    for ax, ref in ((ca, chord), (ta, thick)):
        if abs(amin[ax] - bmin[ax]) > tol_frac * ref or abs(amax[ax] - bmax[ax]) > tol_frac * ref:
            return False
    span = max(amax[sa] - amin[sa], bmax[sa] - bmin[sa])
    gap = max(bmin[sa] - amax[sa], amin[sa] - bmax[sa])
    return abs(gap) <= tol_frac * span


def get_solid(solids, comp):
    """The solid for a component; fuses the second half when A3 merged split half-wings."""
    s = solids[comp["index"]]
    other = comp.get("merged_with")
    if other is None:
        return s
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse
    f = BRepAlgoAPI_Fuse(s, solids[other])
    f.Build()
    if not f.IsDone():
        raise StageError("MERGE_FAILED", "Could not fuse the two half-wing solids")
    return f.Shape()


# ---------------- A3 ----------------
def classify(solids):
    comps = []
    for i, s in enumerate(solids):
        lo, hi = bbox(s)
        size = hi - lo
        order = np.argsort(size)  # thin, mid, long
        d0, d1, d2 = (float(size[j]) for j in order)
        if d2 <= 0:
            kind, ev = "degenerate", "zero extent"
        else:
            thin = d0 / d1 if d1 > 0 else 1
            aspect = d2 / d1 if d1 > 0 else 0
            if thin < 0.3 and aspect >= 2.0:
                kind, ev = "wing_like", f"thickness/chord={thin:.3f}, span/chord={aspect:.2f}"
            elif d1 > 0 and d0 / d1 > 0.6 and d1 / d2 < 0.5:
                kind, ev = "fuselage_like", f"round cross-section ratio {d0 / d1:.2f}, slender {d1 / d2:.2f}"
            else:
                kind, ev = "other", f"dims {d0:.1f}/{d1:.1f}/{d2:.1f}"
        comps.append({"index": i, "kind": kind, "evidence": ev, "bbox_min": lo.tolist(), "bbox_max": hi.tolist(),
                      "size": size.tolist(), "planform_area_bbox": d1 * d2,
                      "volume": volume(s), "thin_axis": AX[order[0]], "chord_axis": AX[order[1]],
                      "span_axis": AX[order[2]]})
    wings = sorted([c for c in comps if c["kind"] == "wing_like"], key=lambda c: -c["planform_area_bbox"])
    if not wings:
        raise StageError("WING_NOT_IDENTIFIED",
                         "No wing-like solid (thin, high aspect ratio) found. A wing fused into a fuselage in a single "
                         "solid, rotated models and surface-only models are not supported.", {"components": comps})
    if len(wings) > 1:
        a, b = wings[0], wings[1]
        if b["planform_area_bbox"] > 0.5 * a["planform_area_bbox"]:
            if _are_split_halves(a, b):
                a["merged_with"] = b["index"]
                lo = np.minimum(a["bbox_min"], b["bbox_min"])
                hi = np.maximum(a["bbox_max"], b["bbox_max"])
                a["bbox_min"], a["bbox_max"] = lo.tolist(), hi.tolist()
                a["size"] = (hi - lo).tolist()
                a["volume"] = a["volume"] + b["volume"]
                a["evidence"] += f"; merged with solid {b['index']} (split half-wings meeting at the span-axis plane)"
                b["role"] = "merged_into_main_wing"
                ax = "XYZ".index(a["span_axis"])
                a["planform_area_bbox"] = a["size"][ax] * a["size"]["XYZ".index(a["chord_axis"])]
                wings = [a]
            else:
                raise StageError("WING_AMBIGUOUS",
                                 f"Solids {a['index']} and {b['index']} are both large wing-like bodies (area ratio "
                                 f"{b['planform_area_bbox'] / a['planform_area_bbox']:.2f}) and are not touching halves of one "
                                 "wing (they differ in chord/thickness or do not meet along the span axis); could be wing+tailplane.",
                                 {"components": comps})
    comps[wings[0]["index"]]["role"] = "main_wing"
    return comps, wings[0]["index"]


# ---------------- sections ----------------
def _edge_points(edge, defl):
    c = BRepAdaptor_Curve(edge)
    d = GCPnts_QuasiUniformDeflection(c, defl)
    if not d.IsDone():
        return None
    pts = [d.Value(i) for i in range(1, d.NbPoints() + 1)]
    return np.array([[p.X(), p.Y(), p.Z()] for p in pts])


def cut_section(solid, span_ax, pos, defl):
    n = [0, 0, 0]
    n[span_ax] = 1
    org = [0, 0, 0]
    org[span_ax] = pos
    sec = BRepAlgoAPI_Section(solid, gp_Pln(gp_Pnt(*org), gp_Dir(*n)))
    sec.Build()
    shape = sec.Shape()
    polys = []
    e = TopExp_Explorer(shape, TopAbs_EDGE)
    while e.More():
        p = _edge_points(TopoDS.Edge(e.Current()), defl)
        if p is not None and len(p) >= 2:
            polys.append(p)
        e.Next()
    return _chain(polys, tol=max(defl * 20, 1e-6))


def _chain(polys, tol):
    """Greedy join of edge polylines into loops. Returns list of (n,3) arrays."""
    polys = list(polys)
    loops = []
    while polys:
        cur = polys.pop(0)
        while polys:
            tail = cur[-1]
            best, bi, brev = None, -1, False
            for i, p in enumerate(polys):
                for rev, end in ((False, p[0]), (True, p[-1])):
                    d = np.linalg.norm(end - tail)
                    if best is None or d < best:
                        best, bi, brev = d, i, rev
            if best is None or best > tol:
                break
            p = polys.pop(bi)
            p = p[::-1] if brev else p
            cur = np.vstack([cur, p[1:]])
        loops.append(cur)
    return loops


def planar(loop, span_ax, chord_ax, thick_ax):
    return np.column_stack([loop[:, chord_ax], loop[:, thick_ax]])


def poly_area(p):
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def outer_loop(loops, ax):
    c = [planar(l, *ax) for l in loops]
    return max(range(len(c)), key=lambda i: poly_area(c[i])), c


def _axes(comp):
    s, c, t = AX.index(comp["span_axis"]), AX.index(comp["chord_axis"]), AX.index(comp["thin_axis"])
    return s, c, t


def chord_len(p2):
    d = np.linalg.norm(p2[:, None, :] - p2[None, :, :], axis=2)
    return float(d.max())


# ---------------- A4 ----------------
def measure(solid, comp, mode="auto", n_probe=9):
    s, c, t = _axes(comp)
    lo, hi = comp["bbox_min"][s], comp["bbox_max"][s]
    span = hi - lo
    defl = max(comp["size"]) * 1e-4
    mid = 0.5 * (lo + hi)
    fr = np.linspace(0.02, 0.98, n_probe)
    probes = []
    for f in fr:
        pos = lo + f * span
        loops = cut_section(solid, s, pos, defl)
        if not loops:
            raise StageError("SECTION_EMPTY", f"No intersection at {AX[s]}={pos:.3f}")
        i, pl = outer_loop(loops, (s, c, t))
        p = pl[i]
        probes.append({"pos": pos, "chord": chord_len(p), "u_min": float(p[:, 0].min()), "u_max": float(p[:, 0].max()),
                       "v_mid": float(0.5 * (p[:, 1].min() + p[:, 1].max()))})
    ch = np.array([q["chord"] for q in probes])
    sym = bool(np.all(np.abs(ch - ch[::-1]) < 0.03 * ch.max()))
    u_c = np.array([0.5 * (q["u_min"] + q["u_max"]) for q in probes])
    sym = sym and bool(np.all(np.abs(u_c - u_c[::-1]) < 0.03 * ch.max()))
    if mode == "auto":
        mode_used = "full_symmetric" if sym else "half"
    elif mode == "full":
        mode_used = "full_symmetric"
        if not sym:
            raise StageError("WING_ASYMMETRIC", "span_mode=full but the wing is not symmetric about the span centre.")
    else:
        mode_used = "half"
    if mode_used == "full_symmetric":
        root, tip_dir, half = mid, +1, span / 2
    else:  # root = end with larger chord
        if ch[0] >= ch[-1]:
            root, tip_dir = lo, +1
        else:
            root, tip_dir = hi, -1
        half = span
    return {"span_axis": AX[s], "chord_axis": AX[c], "thin_axis": AX[t], "mode": mode_used,
            "symmetric_about_centre": sym, "total_span": span, "half_span": half, "root_pos": root,
            "tip_direction": tip_dir, "root_chord": float(ch[len(ch) // 2] if mode_used == "full_symmetric" else ch[0 if tip_dir == 1 else -1]),
            "tip_chord": float(ch[-1] if mode_used == "half" and tip_dir == 1 else (ch[0] if mode_used == "half" else ch[-1])),
            "probes": probes,
            "assumptions": ["CAD axes are aligned with the wing (no rotation search)", "+thickness axis is up",
                            f"span mode '{mode_used}' ({'assumed' if mode == 'auto' else 'configured'})"]}


# ---------------- A5 ----------------
def default_stations(n):
    eps = 0.002
    f = np.linspace(0, 1, n)
    return [float(max(eps, min(1 - eps, x))) for x in f], [float(x) for x in f]


def extract_sections(solid, comp, meas, n_sections):
    s, c, t = _axes(comp)
    defl = max(comp["size"]) * 1e-4
    cut_f, nominal_f = default_stations(n_sections)
    out = []
    for cf, nf in zip(cut_f, nominal_f):
        pos = meas["root_pos"] + meas["tip_direction"] * cf * meas["half_span"]
        entry = {"nominal_fraction": nf, "cut_fraction": cf, "span_position": pos,
                 "y_from_root": nf * meas["half_span"]}
        loops = cut_section(solid, s, pos, defl)
        if not loops:
            entry.update(status="FAIL", reason="no intersection")
            out.append(entry)
            continue
        i, pl = outer_loop(loops, (s, c, t))
        p = pl[i]
        entry.update(status="PASS", loop_count=len(loops), points=p.tolist(), n_points=len(p),
                     closed=bool(np.linalg.norm(p[0] - p[-1]) < defl * 50))
        out.append(entry)
    return out


# ---------------- A6 ----------------
def cosine_x(n):
    th = np.linspace(0, math.pi, n)
    return 0.5 * (1 - np.cos(th))


def reconstruct(pts, u_dir=None, n_per_side=61):
    """pts: (n,2) closed section in (chord-axis, thickness-axis). Returns dict.
    u_dir: +1 / -1 chord orientation (LE at u_min / u_max); None = detect."""
    p = np.asarray(pts, float)
    if u_dir is None:
        u_dir = detect_direction(p)
    q = p.copy()
    q[:, 0] *= u_dir
    le = q[np.argmin(q[:, 0])]
    # TE reference = farthest point from LE
    far = q[np.argmax(np.linalg.norm(q - le, axis=1))]
    # use mean of extreme-x points to be robust for blunt TE
    xmax = q[:, 0].max()
    te = q[q[:, 0] > xmax - 1e-3 * (xmax - le[0])].mean(axis=0) if xmax > le[0] else far
    vec = te - le
    chord = float(np.linalg.norm(vec))
    ang = math.atan2(vec[1], vec[0])
    r = np.array([[math.cos(-ang), -math.sin(-ang)], [math.sin(-ang), math.cos(-ang)]])
    z = (q - le) @ r.T / chord
    # LE index after normalisation
    # Closed outline: split into the two LE->TE branches independent of where the loop starts.
    if np.linalg.norm(z[0] - z[-1]) < 1e-6 * max(1.0, np.abs(z).max()):
        z = z[:-1]
    i_le = int(np.argmin(z[:, 0]))
    n = len(z)
    zr = np.roll(z, -i_le, axis=0)  # starts at LE
    i_te = int(np.argmax(zr[:, 0]))
    a = zr[: i_te + 1]  # LE -> TE one way round
    b = np.vstack([zr[i_te:], zr[:1]])  # TE -> LE the other way round
    def side(br):
        o = np.argsort(br[:, 0], kind="stable")
        br = br[o]
        return br
    a, b = side(a), side(b)
    up, lo_ = (a, b) if a[:, 1].mean() >= b[:, 1].mean() else (b, a)
    xs = cosine_x(n_per_side)
    def interp(br):
        x, y = br[:, 0], br[:, 1]
        keep = np.concatenate([[True], np.diff(x) > 1e-12])
        x, y = x[keep], y[keep]
        yy = np.interp(xs, x, y)
        yy[0] = 0.0 if abs(yy[0]) < 1e-3 else yy[0]
        return yy
    yu, yl = interp(up), interp(lo_)
    yu[0] = yl[0] = 0.5 * (yu[0] + yl[0])  # shared LE point
    upper = np.column_stack([xs, yu])
    lower = np.column_stack([xs, yl])
    coords = np.vstack([upper[::-1], lower[1:]])  # Selig: TE upper -> LE -> TE lower
    dev = deviation(z, coords)
    return {"coords": coords, "chord": chord, "twist_deg": -math.degrees(ang), "le_xy_raw": [float(le[0] * u_dir), float(le[1])],
            "u_dir": int(u_dir), "max_deviation_over_chord": dev,
            "max_thickness_over_chord": float((yu - yl).max()),
            "te_thickness_over_chord": float(yu[-1] - yl[-1])}


def detect_direction(p):
    """Leading edge is the blunt end: thicker at 5% chord than at 95%."""
    def thick_at(q, frac):
        u0, u1 = q[:, 0].min(), q[:, 0].max()
        x = u0 + frac * (u1 - u0)
        w = (u1 - u0) * 0.03
        m = q[np.abs(q[:, 0] - x) < w]
        return float(m[:, 1].max() - m[:, 1].min()) if len(m) else 0.0
    return +1 if thick_at(p, 0.05) >= thick_at(p, 0.95) else -1


def deviation(z, coords):
    """Max distance from original (normalised) points to reconstructed polyline, over chord."""
    a, b = coords[:-1], coords[1:]
    ab = b - a
    den = np.maximum((ab ** 2).sum(1), 1e-18)
    best = np.full(len(z), np.inf)
    for i in range(len(a)):
        t = np.clip(((z - a[i]) @ ab[i]) / den[i], 0, 1)
        d = np.linalg.norm(z - (a[i] + t[:, None] * ab[i]), axis=1)
        best = np.minimum(best, d)
    return float(best.max())
