"""Localize a DXF-vs-STEP disagreement: show WHICH strokes differ, not just how many.

The coverage numbers in verify_pair.py say how much two renders disagree; this says where,
which is what tells you whether a residual is a filtering bug, a misclassification, or
geometry one kernel draws and the other does not. Reading these maps is what showed that
the hidden-line residual sits at blend corners rather than alongside visible lines
(see FIX_IDEAS.md §3).

Colours, over a dim grey of everything the two renders agree on:
  magenta  our red the DXF has nothing near      (hidden strokes we invent)
  cyan     DXF red we have nothing near          (hidden strokes we miss)
  yellow   our white the DXF has nothing near    (visible strokes we invent)
  green    DXF white we have nothing near        (visible strokes we miss)

Usage: python diff_map.py <examples_dir> <stem> [out.png] [x0,y0,x1,y1]
"""
import os
import sys

import numpy as np
from PIL import Image
from scipy import ndimage

from dxf_render import dxf_polylines, render
from step_render import step_polylines
from compare import masks

TOL = 2  # px, same slack as the coverage metric in compare.py

AGREE = (70, 70, 70)
FP_HIDDEN = (255, 0, 255)
FN_HIDDEN = (0, 200, 255)
FP_VISIBLE = (255, 230, 0)
FN_VISIBLE = (0, 220, 0)


def diff_masks(dxf_img, step_img):
    """(false positive, false negative) masks for hidden and visible strokes."""
    dw, dr = masks(dxf_img)
    sw, sr = masks(step_img)

    def near(m):
        return ndimage.binary_dilation(m, iterations=TOL)

    return {
        "fp_hidden": sr & ~near(dr),
        "fn_hidden": dr & ~near(sr),
        "fp_visible": sw & ~near(dw),
        "fn_visible": dw & ~near(sw),
        "all": sw | sr | dw | dr,
    }


def diff_image(dxf_img, step_img):
    d = diff_masks(dxf_img, step_img)
    wrong = d["fp_hidden"] | d["fn_hidden"] | d["fp_visible"] | d["fn_visible"]
    h, w = d["all"].shape
    out = np.zeros((h, w, 3), dtype=np.uint8)
    out[d["all"] & ~wrong] = AGREE
    for key, colour in (("fn_visible", FN_VISIBLE), ("fn_hidden", FN_HIDDEN),
                        ("fp_visible", FP_VISIBLE), ("fp_hidden", FP_HIDDEN)):
        out[d[key]] = colour
    return Image.fromarray(out), d


def main(example_dir, stem, out_png=None, crop=None):
    dxf_img = render(list(dxf_polylines(os.path.join(example_dir, stem + ".dxf"))), None)
    step_img = render(step_polylines(os.path.join(example_dir, stem + ".step")), None)
    img, d = diff_image(dxf_img, step_img)
    counts = {k: int(v.sum()) for k, v in d.items() if k != "all"}
    agreed = int((d["all"].sum()) - sum(counts.values()))
    print(f"{stem}: agree={agreed} " + " ".join(f"{k}={v}" for k, v in counts.items()))
    if crop:
        img = img.crop(crop)
    path = out_png or f"diff_{stem}.png"
    img.save(path)
    print(f"wrote {path}")


if __name__ == "__main__":
    crop = tuple(int(v) for v in sys.argv[4].split(",")) if len(sys.argv) > 4 else None
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None, crop)
