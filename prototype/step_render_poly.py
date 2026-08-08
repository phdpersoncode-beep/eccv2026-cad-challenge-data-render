"""Mesh-based HLR fallback (HLRBRep_PolyAlgo) for parts where exact HLR is pathological.

Same normalization, views, coincidence filter and rasterization as step_render;
only the hidden-line engine differs (polygonal, from a triangulation).
"""
import sys
import time

from OCP.HLRBRep import HLRBRep_PolyAlgo, HLRBRep_PolyHLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from OCP.BRepMesh import BRepMesh_IncrementalMesh

from step_render import (VIEWS, normalized_shape, rotate_for_view, edges_of,
                         discretize, filter_coincident_hidden)
from dxf_render import render

MESH_DEFLECTION = 0.05  # mm on the 100 mm model


def poly_hlr_view(shape_ocp):
    BRepMesh_IncrementalMesh(shape_ocp, MESH_DEFLECTION)
    algo = HLRBRep_PolyAlgo()
    algo.Load(shape_ocp)
    algo.Projector(HLRAlgo_Projector(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))))
    algo.Update()
    hlr = HLRBRep_PolyHLRToShape()
    hlr.Update(algo)
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


def step_polylines_poly(step_file):
    base = normalized_shape(step_file)
    polys = []
    for name, (R, center) in VIEWS.items():
        rotated = rotate_for_view(base.wrapped, R)
        vis, hid = poly_hlr_view(rotated)
        hid = filter_coincident_hidden(vis, hid)
        cx, cy = center
        for pts in vis:
            polys.append(([(x + cx, y + cy) for x, y in pts], "visible"))
        for pts in hid:
            polys.append(([(x + cx, y + cy) for x, y in pts], "hidden"))
        print(f"  {name}: {len(vis)} visible, {len(hid)} hidden (poly HLR)", file=sys.stderr)
    return polys


if __name__ == "__main__":
    t0 = time.time()
    step_file, out_png = sys.argv[1], sys.argv[2]
    render(step_polylines_poly(step_file), out_png)
    print(f"wrote {out_png} in {time.time()-t0:.1f}s")
