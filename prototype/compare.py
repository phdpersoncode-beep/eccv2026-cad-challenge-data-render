"""Compare DXF-derived and STEP-derived renders for all example pairs.

Metrics (per pair, on stroke pixels):
  - coverage with 2px tolerance (dilated-target recall both directions), per class:
    * any stroke (white or red)
    * visible only (white)
    * hidden only (red)
"""
import glob
import os
import sys
import numpy as np
from PIL import Image

from dxf_render import dxf_polylines, render
from step_render import step_polylines

SP = os.getcwd()  # comparison renders (cmp_*.png) are written to the current directory


def masks(img):
    a = np.asarray(img)
    white = (a[..., 0] > 128) & (a[..., 1] > 128) & (a[..., 2] > 128)
    red = (a[..., 0] > 128) & (a[..., 1] < 128) & (a[..., 2] < 128)
    return white, red


def dilate(m, r=2):
    from scipy import ndimage
    return ndimage.binary_dilation(m, iterations=r)


def cov(a, b, tol=2):
    """fraction of a's pixels within tol pixels of b"""
    if a.sum() == 0:
        return 1.0
    if b.sum() == 0:
        return 0.0
    return (a & dilate(b, tol)).sum() / a.sum()


def sym_coverage(a, b, tol=2):
    """fraction of a's pixels within tol of b, and vice versa; return min of both"""
    if a.sum() == 0 and b.sum() == 0:
        return 1.0
    if a.sum() == 0 or b.sum() == 0:
        return 0.0
    return min(cov(a, b, tol), cov(b, a, tol))


def main(example_dir, limit=None):
    steps = sorted(glob.glob(os.path.join(example_dir, "0000*.step")))
    if limit:
        steps = steps[:limit]
    rows = []
    for sf in steps:
        stem = os.path.splitext(os.path.basename(sf))[0]
        df = os.path.join(example_dir, stem + ".dxf")
        if not os.path.exists(df):
            continue
        try:
            dimg = render(list(dxf_polylines(df)), os.path.join(SP, f"cmp_dxf_{stem}.png"))
            simg = render(step_polylines(sf), os.path.join(SP, f"cmp_step_{stem}.png"))
        except Exception as ex:
            print(f"{stem}: ERROR {ex}")
            continue
        dw, dr = masks(dimg)
        sw, sr = masks(simg)
        c_all = sym_coverage(dw | dr, sw | sr)
        c_vis = sym_coverage(dw, sw)
        c_hid = sym_coverage(dr, sr)
        rows.append((stem, c_all, c_vis, c_hid))
        print(f"{stem}: all={c_all:.3f} visible={c_vis:.3f} hidden={c_hid:.3f} "
              f"| hid recall={cov(dr, sr):.3f} precision={cov(sr, dr):.3f}")
    arr = np.array([[r[1], r[2], r[3]] for r in rows])
    print(f"\nMEAN over {len(rows)}: all={arr[:,0].mean():.3f} visible={arr[:,1].mean():.3f} hidden={arr[:,2].mean():.3f}")
    print(f"MEDIAN: all={np.median(arr[:,0]):.3f} visible={np.median(arr[:,1]):.3f} hidden={np.median(arr[:,2]):.3f}")
    print(f"MIN: all={arr[:,0].min():.3f} visible={arr[:,1].min():.3f} hidden={arr[:,2].min():.3f}")
    worst = sorted(rows, key=lambda r: r[1])[:5]
    print("worst (by all):", [(r[0], round(r[1], 3)) for r in worst])


if __name__ == "__main__":
    limit = int(sys.argv[2]) if len(sys.argv) > 2 else None
    main(sys.argv[1], limit)
