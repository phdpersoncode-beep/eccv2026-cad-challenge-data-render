"""Prototype: STEP -> third-angle three-view drawing PNG via OCC hidden line removal.

Replicates the SolidWorks pipeline geometry:
  - normalize: bbox center -> origin, uniform scale so max dimension = 100 mm
  - front  view: screen (x, y),  depth z, view center (83, 57) mm
  - top    view: screen (x, -z), depth y, view center (83, 158) mm
  - right  view: screen (-z, y), depth x, view center (223, 57) mm
  - visible = VCompound + OutLineVCompound, hidden = HCompound + OutLineHCompound
    (tangent/Rg1 edges excluded == SolidWorks 'tangent edges hidden')
"""
import sys
import cadquery as cq
from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir, gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE
from OCP.TopoDS import TopoDS

from dxf_render import render  # reuse rasterizer

TARGET_MAX_DIM = 100.0  # mm, matches drawing_model_target_max_dimension_m = 0.1
VIEWS = {
    # name: (rotation rows applied as p' = p . R  [row-vector convention], center mm)
    "front": (((1, 0, 0), (0, 1, 0), (0, 0, 1)), (83.0, 57.0)),
    "top":   (((1, 0, 0), (0, 0, 1), (0, -1, 0)), (83.0, 158.0)),
    "right": (((0, 0, 1), (0, 1, 0), (-1, 0, 0)), (223.0, 57.0)),
}
DEFLECTION = 0.05  # mm discretization tolerance


def _collect(shape_ocp, kind, caster):
    exp = TopExp_Explorer(shape_ocp, kind)
    out = []
    while exp.More():
        out.append(cq.Shape.cast(caster(exp.Current())))
        exp.Next()
    return out


def normalization(step_file, report=None):
    """Import STEP -> (shape, bbox center, scale to TARGET_MAX_DIM), geometry untouched.

    Uses solids only (stray shells/faces in some corpus files poison the bbox) and the
    exact BRepBndLib.AddOptimal bounding box (tessellation-independent).

    The centering and scaling are deliberately *not* baked into the B-rep. Applying them
    (cq's translate/scale, like any BRepBuilderAPI_Transform with copy=True) reruns the
    shape through BRepTools_Modifier, and on some corpus parts the rebuilt faces make
    exact HLR abort and return an empty view (see hlr_view / step_polylines). Orthographic
    projection commutes with both, so step_polylines applies them to the 2D result instead.
    """
    from OCP.TopAbs import TopAbs_SOLID, TopAbs_SHELL
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    vals = cq.importers.importStep(step_file).vals()
    parts = []
    for v in vals:
        parts.extend(_collect(v.wrapped, TopAbs_SOLID, TopoDS.Solid_s))
    if not parts:  # fall back to shells for non-solid models
        for v in vals:
            parts.extend(_collect(v.wrapped, TopAbs_SHELL, TopoDS.Shell_s))

    def finite_box(shape):
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape.wrapped, box, True, False)
        if box.IsVoid():
            return None
        vs = box.Get()
        if max(abs(c) for c in vs) > 1e6:
            return None
        return vs

    # drop bodies whose own bbox is unbounded (corrupt surfaces in some corpus files)
    kept = [(p, fb) for p in parts if (fb := finite_box(p)) is not None]
    if not kept:
        raise ValueError("no solids or shells with a finite bounding box")
    dropped = len(parts) - len(kept)
    if dropped:  # never drop geometry silently — a dropped body can be a whole view
        print(f"  normalize: dropped {dropped} of {len(parts)} bodies (non-finite bbox)",
              file=sys.stderr)
    if report is not None:
        report["dropped_bodies"] = dropped
    parts = [p for p, _ in kept]
    boxes = [fb for _, fb in kept]
    shape = parts[0] if len(parts) == 1 else cq.Compound.makeCompound(parts)

    xmin = min(b[0] for b in boxes); ymin = min(b[1] for b in boxes); zmin = min(b[2] for b in boxes)
    xmax = max(b[3] for b in boxes); ymax = max(b[4] for b in boxes); zmax = max(b[5] for b in boxes)
    max_dim = max(xmax - xmin, ymax - ymin, zmax - zmin)
    if not (1e-9 < max_dim < 1e6):
        raise ValueError(f"degenerate bounding box (max dim {max_dim:g})")
    center = ((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2)
    return shape, center, TARGET_MAX_DIM / max_dim


def normalized_shape(step_file, report=None):
    """normalization() with the centering and scaling baked into the B-rep.

    Kept for the poly-HLR path and external callers; the exact-HLR path must not use it
    (the rebuild it triggers is what empties views — see normalization).
    """
    shape, center, scale = normalization(step_file, report)
    return shape.translate((-center[0], -center[1], -center[2])).scale(scale)


def rotate_for_view(shape_ocp, R_rows, copy=False):
    # p' = p . R (row vector)  ==  p' = R^T p (column vector); gp_Trsf wants row-major of the matrix M with p' = M p
    M = [[R_rows[j][i] for j in range(3)] for i in range(3)]  # transpose
    tr = gp_Trsf()
    tr.SetValues(M[0][0], M[0][1], M[0][2], 0,
                 M[1][0], M[1][1], M[1][2], 0,
                 M[2][0], M[2][1], M[2][2], 0)
    # copy=False keeps the rotation as a TopLoc location: the B-rep is not rebuilt, so HLR
    # sees the surfaces the STEP file actually carries (and we skip a modifier per view)
    return BRepBuilderAPI_Transform(shape_ocp, tr, copy).Shape()


def edges_of(compound):
    if compound is None:
        return
    exp = TopExp_Explorer(compound, TopAbs_EDGE)
    while exp.More():
        yield TopoDS.Edge_s(exp.Current())
        exp.Next()


UNIFORM_SAMPLES = 32  # points used when deflection sampling refuses a curve


def discretize(edge, deflection=DEFLECTION):
    """Edge -> [(x, y), ...] at `deflection` sagitta, in the edge's own units."""
    curve = BRepAdaptor_Curve(edge)
    disc = GCPnts_QuasiUniformDeflection(curve, deflection)
    if disc.IsDone():
        return [(disc.Value(i).X(), disc.Value(i).Y()) for i in range(1, disc.NbPoints() + 1)]
    # deflection sampling gives up on some curves (degenerate parametrization); sample the
    # parameter range uniformly instead of dropping the edge from the drawing
    u0, u1 = curve.FirstParameter(), curve.LastParameter()
    if not (u1 > u0):
        print(f"    discretize: unusable curve range [{u0:g}, {u1:g}] — edge skipped", file=sys.stderr)
        return []
    pts = [curve.Value(u0 + (u1 - u0) * i / UNIFORM_SAMPLES) for i in range(UNIFORM_SAMPLES + 1)]
    return [(p.X(), p.Y()) for p in pts]


def hlr_view(shape_ocp, deflection=DEFLECTION):
    """Run HLR looking from +Z; returns (visible_edges, hidden_edges) as point lists in view XY."""
    algo = HLRBRep_Algo()
    algo.Add(shape_ocp)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))))
    algo.Update()
    algo.Hide()
    hlr = HLRBRep_HLRToShape(algo)
    vis, hid = [], []
    for name, getter, sink in (("VCompound", hlr.VCompound, vis),
                               ("OutLineVCompound", hlr.OutLineVCompound, vis),
                               ("HCompound", hlr.HCompound, hid),
                               ("OutLineHCompound", hlr.OutLineHCompound, hid)):
        try:
            comp = getter()
        except Exception as ex:  # an empty compound is normal, a raising getter is not
            print(f"    HLR {name} failed: {type(ex).__name__}: {ex}", file=sys.stderr)
            continue
        if comp is None or comp.IsNull():
            continue
        for edge in edges_of(comp):
            pts = discretize(edge, deflection)
            if len(pts) >= 2:
                sink.append(pts)
    return vis, hid


COINCIDENCE_EPS = 0.15   # mm: hidden points closer than this to visible geometry count as coincident
COINCIDENCE_FRAC = 0.90  # drop hidden edge if this fraction of its samples is coincident
SAMPLE_STEP = 0.1        # mm resampling step for the coincidence test


def _resample(pts, step):
    import numpy as np
    out = [pts[0]]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
        n = max(1, int(d / step))
        for i in range(1, n + 1):
            t = i / n
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    return np.array(out)


def filter_coincident_hidden(vis, hid):
    """Drop hidden edges whose projection coincides with visible geometry.

    SolidWorks drawings merge a hidden edge into the visible edge that covers it;
    OCC HLR reports both. Without this filter the render grows red fringes along
    white lines that the DXF ground truth does not have.
    """
    import numpy as np
    from scipy.spatial import cKDTree
    if not vis or not hid:
        return hid
    vis_pts = np.vstack([_resample(p, SAMPLE_STEP) for p in vis])
    tree = cKDTree(vis_pts)
    kept = []
    for pts in hid:
        s = _resample(pts, SAMPLE_STEP)
        d, _ = tree.query(s, k=1)
        if (d < COINCIDENCE_EPS).mean() < COINCIDENCE_FRAC:
            kept.append(pts)
    return kept


def step_polylines(step_file, report=None):
    """STEP -> [(points_mm, 'visible'|'hidden'), ...] placed on the sheet, three views.

    Pass a dict as `report` to get the diagnostics a corpus harness should flag on:
    'dropped_bodies', 'empty_views' (views with no visible edge — a silently wrong render)
    and 'poly_fallback_views' (views that only came out of the mesh HLR retry).
    """
    if report is not None:
        report.update(dropped_bodies=0, empty_views=[], poly_fallback_views=[])
    shape, center, scale = normalization(step_file, report)
    # HLR runs on the un-normalized B-rep, so the deflection has to be un-normalized too
    deflection = DEFLECTION / scale
    has_faces = TopExp_Explorer(shape.wrapped, TopAbs_FACE).More()
    polys = []
    for name, (R, view_center) in VIEWS.items():
        rotated = rotate_for_view(shape.wrapped, R)
        vis, hid = hlr_view(rotated, deflection)
        if not vis and has_faces:
            # exact HLR abandons the whole view when it chokes on one face; the mesh engine
            # is immune, so retry there rather than emit a view that is silently empty
            print(f"  {name}: exact HLR returned no visible edge — retrying with poly HLR",
                  file=sys.stderr)
            from step_render_poly import poly_hlr_view  # local: that module imports this one
            vis, hid = poly_hlr_view(rotated, deflection)
            if vis and report is not None:
                report["poly_fallback_views"].append(name)
        if not vis and has_faces:
            print(f"  {name}: VIEW IS EMPTY — render is incomplete", file=sys.stderr)
            if report is not None:
                report["empty_views"].append(name)
        # normalization is applied here, not to the B-rep: an orthographic projection
        # commutes with a uniform scale about the origin and with a translation
        ox = sum(center[j] * R[j][0] for j in range(3))
        oy = sum(center[j] * R[j][1] for j in range(3))
        cx, cy = view_center
        vis = [[((x - ox) * scale + cx, (y - oy) * scale + cy) for x, y in pts] for pts in vis]
        hid = [[((x - ox) * scale + cx, (y - oy) * scale + cy) for x, y in pts] for pts in hid]
        hid = filter_coincident_hidden(vis, hid)
        polys.extend((pts, "visible") for pts in vis)
        polys.extend((pts, "hidden") for pts in hid)
        print(f"  {name}: {len(vis)} visible, {len(hid)} hidden edges (after coincidence filter)",
              file=sys.stderr)
    return polys


if __name__ == "__main__":
    step_file, out_png = sys.argv[1], sys.argv[2]
    polys = step_polylines(step_file)
    render(polys, out_png)
    print(f"wrote {out_png}")
