"""Prototype: rasterize SolidWorks-exported drawing DXF to PNG.

Black background, white = visible edges (linetype Continuous on layer 0),
red = hidden edges (linetype HIDDEN on layer 0). Center-mark blocks
(layer 10 INSERTs) are ignored.
"""
import sys
import ezdxf
from ezdxf import path as ezpath
import numpy as np
from PIL import Image, ImageDraw

SHEET_W, SHEET_H = 297.0, 210.0  # mm, A4 landscape
PX_PER_MM = 4.0                  # 1188 x 840 output
FLATTEN_DIST = 0.05              # mm, max sagitta for curve flattening

VISIBLE = (255, 255, 255)
HIDDEN = (255, 0, 0)


def dxf_polylines(dxf_file):
    """Yield (points_mm, kind) with kind in {'visible','hidden'} from modelspace layer-0 geometry."""
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()
    for e in msp:
        t = e.dxftype()
        if t == "INSERT":
            continue  # center marks and other annotation blocks
        if e.dxf.layer != "0":
            continue
        lt = e.dxf.get("linetype", "Continuous")
        kind = "hidden" if lt.upper() == "HIDDEN" else "visible"
        try:
            p = ezpath.make_path(e)
        except Exception as ex:
            print(f"  skip {t}: {ex}", file=sys.stderr)
            continue
        pts = [(v.x, v.y) for v in p.flattening(FLATTEN_DIST)]
        if len(pts) >= 2:
            yield pts, kind


def render(polylines, out_png, line_px=2):
    w, h = int(SHEET_W * PX_PER_MM), int(SHEET_H * PX_PER_MM)
    img = Image.new("RGB", (w, h), (0, 0, 0))
    draw = ImageDraw.Draw(img)

    def to_px(pt):
        x, y = pt
        return (x * PX_PER_MM, (SHEET_H - y) * PX_PER_MM)  # flip y

    # draw hidden first so visible wins at crossings (matches drawing convention)
    for kind_pass, color in (("hidden", HIDDEN), ("visible", VISIBLE)):
        for pts, kind in polylines:
            if kind != kind_pass:
                continue
            draw.line([to_px(p) for p in pts], fill=color, width=line_px, joint="curve")
    img.save(out_png)
    return img


if __name__ == "__main__":
    dxf_file, out_png = sys.argv[1], sys.argv[2]
    polys = list(dxf_polylines(dxf_file))
    nvis = sum(1 for _, k in polys if k == "visible")
    nhid = sum(1 for _, k in polys if k == "hidden")
    print(f"{dxf_file}: {nvis} visible, {nhid} hidden polylines")
    render(polys, out_png)
    print(f"wrote {out_png}")
