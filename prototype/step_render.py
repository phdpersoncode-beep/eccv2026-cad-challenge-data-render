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
from OCP.TopAbs import TopAbs_EDGE
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


def normalized_shape(step_file):
    """Import STEP -> compound of solids, bbox-centered at origin, max dim = TARGET_MAX_DIM.

    Uses solids only (stray shells/faces in some corpus files poison the bbox) and the
    exact BRepBndLib.AddOptimal bounding box (tessellation-independent).
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
    if not parts:
        raise ValueError("no solids or shells in STEP")
    shape = parts[0] if len(parts) == 1 else cq.Compound.makeCompound(parts)

    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, True, False)
    if box.IsVoid():
        raise ValueError("void bounding box")
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    max_dim = max(xmax - xmin, ymax - ymin, zmax - zmin)
    if not (1e-9 < max_dim < 1e6):
        raise ValueError(f"degenerate bounding box (max dim {max_dim:g})")
    center = ((xmin + xmax) / 2, (ymin + ymax) / 2, (zmin + zmax) / 2)
    moved = shape.translate((-center[0], -center[1], -center[2]))
    return moved.scale(TARGET_MAX_DIM / max_dim)


def rotate_for_view(shape_ocp, R_rows):
    # p' = p . R (row vector)  ==  p' = R^T p (column vector); gp_Trsf wants row-major of the matrix M with p' = M p
    M = [[R_rows[j][i] for j in range(3)] for i in range(3)]  # transpose
    tr = gp_Trsf()
    tr.SetValues(M[0][0], M[0][1], M[0][2], 0,
                 M[1][0], M[1][1], M[1][2], 0,
                 M[2][0], M[2][1], M[2][2], 0)
    return BRepBuilderAPI_Transform(shape_ocp, tr, True).Shape()


def edges_of(compound):
    if compound is None:
        return
    exp = TopExp_Explorer(compound, TopAbs_EDGE)
    while exp.More():
        yield TopoDS.Edge_s(exp.Current())
        exp.Next()


def discretize(edge):
    curve = BRepAdaptor_Curve(edge)
    disc = GCPnts_QuasiUniformDeflection(curve, DEFLECTION)
    if not disc.IsDone():
        return []
    return [(disc.Value(i).X(), disc.Value(i).Y()) for i in range(1, disc.NbPoints() + 1)]


def hlr_view(shape_ocp):
    """Run HLR looking from +Z; returns (visible_edges, hidden_edges) as point lists in view XY mm."""
    algo = HLRBRep_Algo()
    algo.Add(shape_ocp)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))))
    algo.Update()
    algo.Hide()
    hlr = HLRBRep_HLRToShape(algo)
    vis, hid = [], []
    for getter, sink in ((hlr.VCompound, vis), (hlr.OutLineVCompound, vis),
                         (hlr.HCompound, hid), (hlr.OutLineHCompound, hid)):
        try:
            comp = getter()
        except Exception:
            continue
        if comp is None:
            continue
        for edge in edges_of(comp):
            pts = discretize(edge)
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


def step_polylines(step_file):
    base = normalized_shape(step_file)
    polys = []
    for name, (R, center) in VIEWS.items():
        rotated = rotate_for_view(base.wrapped, R)
        vis, hid = hlr_view(rotated)
        hid = filter_coincident_hidden(vis, hid)
        cx, cy = center
        for pts in vis:
            polys.append(([(x + cx, y + cy) for x, y in pts], "visible"))
        for pts in hid:
            polys.append(([(x + cx, y + cy) for x, y in pts], "hidden"))
        print(f"  {name}: {len(vis)} visible, {len(hid)} hidden edges (after coincidence filter)",
              file=sys.stderr)
    return polys


if __name__ == "__main__":
    step_file, out_png = sys.argv[1], sys.argv[2]
    polys = step_polylines(step_file)
    render(polys, out_png)
    print(f"wrote {out_png}")
