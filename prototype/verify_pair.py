"""Verify one DXF/STEP pair: render both, compare, print one CSV line to stdout.

CSV: stem,status,all,visible,hidden,hid_recall,hid_precision,seconds,n_vis,n_hid
Optionally saves the two renders with --save-png.
"""
import os
import sys
import time

SP = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SP)


VIEW_CENTERS = [(83.0, 57.0), (83.0, 158.0), (223.0, 57.0)]  # front, top, right


def _view_bboxes(polys):
    """Visible-only 2D bbox per view; polylines assigned to nearest view center."""
    import numpy as np
    pts_by_view = [[], [], []]
    for pts, kind in polys:
        if kind != "visible":
            continue
        p = np.asarray(pts)
        i = int(np.argmin([((p.mean(axis=0) - c) ** 2).sum() for c in VIEW_CENTERS]))
        pts_by_view[i].append(p)
    out = []
    for chunks in pts_by_view:
        if not chunks:
            out.append(None)
            continue
        allp = np.vstack(chunks)
        out.append((allp.min(axis=0), allp.max(axis=0)))
    return out


def fit_similarity(dpolys, spolys):
    """Fit per-part uniform scale s and 3D offset T aligning the STEP render to the DXF.

    Models the SolidWorks approximate-bbox normalization error: a single 3D similarity
    (no rotation). Offsets per view relate to T as front=(Tx,Ty), top=(Tx,-Tz), right=(-Tz,Ty).
    """
    import numpy as np
    db, sb = _view_bboxes(dpolys), _view_bboxes(spolys)
    ratios, tv = [], [None, None, None]
    for i in range(3):
        if db[i] is None or sb[i] is None:
            continue
        dsize, ssize = db[i][1] - db[i][0], sb[i][1] - sb[i][0]
        for a, b in zip(dsize, ssize):
            if a > 1 and b > 1:
                ratios.append(a / b)
    s = float(np.median(ratios)) if ratios else 1.0
    for i in range(3):
        if db[i] is None or sb[i] is None:
            continue
        c = np.asarray(VIEW_CENTERS[i])
        dc, sc = (db[i][0] + db[i][1]) / 2, (sb[i][0] + sb[i][1]) / 2
        tv[i] = dc - (c + s * (sc - c))
    tx = np.mean([v for v in (tv[0][0] if tv[0] is not None else None,
                              tv[1][0] if tv[1] is not None else None) if v is not None] or [0])
    ty = np.mean([v for v in (tv[0][1] if tv[0] is not None else None,
                              tv[2][1] if tv[2] is not None else None) if v is not None] or [0])
    tz = np.mean([v for v in (-tv[1][1] if tv[1] is not None else None,
                              -tv[2][0] if tv[2] is not None else None) if v is not None] or [0])
    return s, (float(tx), float(ty), float(tz))


def apply_similarity(polys, s, T):
    """Apply fitted scale+offset to STEP polylines (per view, consistent with one 3D motion)."""
    import numpy as np
    tx, ty, tz = T
    t_per_view = [np.array([tx, ty]), np.array([tx, -tz]), np.array([-tz, ty])]
    out = []
    for pts, kind in polys:
        p = np.asarray(pts)
        i = int(np.argmin([((p.mean(axis=0) - c) ** 2).sum() for c in VIEW_CENTERS]))
        c = np.asarray(VIEW_CENTERS[i])
        out.append(((c + s * (p - c) + t_per_view[i]).tolist(), kind))
    return out


def main(example_dir, stem, save_png=False):
    t0 = time.time()
    import numpy as np
    from dxf_render import dxf_polylines, render
    from step_render import step_polylines
    from compare import masks, sym_coverage, cov

    dxf = os.path.join(example_dir, stem + ".dxf")
    step = os.path.join(example_dir, stem + ".step")
    out_d = os.path.join(os.getcwd(), f"cmp_dxf_{stem}.png") if save_png else None
    out_s = os.path.join(os.getcwd(), f"cmp_step_{stem}.png") if save_png else None

    dpolys = list(dxf_polylines(dxf))
    dimg = render(dpolys, out_d)
    spolys = step_polylines(step)
    simg = render(spolys, out_s)
    dw, dr = masks(dimg)
    sw, sr = masks(simg)
    c_all = sym_coverage(dw | dr, sw | sr)
    c_vis = sym_coverage(dw, sw)
    c_hid = sym_coverage(dr, sr)
    n_vis = sum(1 for _, k in spolys if k == "visible")
    n_hid = sum(1 for _, k in spolys if k == "hidden")

    # similarity-aligned metrics: separate SW approximate-bbox normalization error
    # (per-part scale + 3D offset) from true structural mismatch
    s, T = fit_similarity(dpolys, spolys)
    aimg = render(apply_similarity(spolys, s, T),
                  out_s.replace(".png", "_adj.png") if save_png else None)
    aw, ar = masks(aimg)
    a_all = sym_coverage(dw | dr, aw | ar)
    a_vis = sym_coverage(dw, aw)
    a_hid = sym_coverage(dr, ar)
    t_mag = (T[0] ** 2 + T[1] ** 2 + T[2] ** 2) ** 0.5

    print(f"{stem},ok,{c_all:.4f},{c_vis:.4f},{c_hid:.4f},"
          f"{cov(dr, sr):.4f},{cov(sr, dr):.4f},{time.time()-t0:.1f},{n_vis},{n_hid},"
          f"{s:.4f},{t_mag:.2f},{a_all:.4f},{a_vis:.4f},{a_hid:.4f}")


if __name__ == "__main__":
    example_dir, stem = sys.argv[1], sys.argv[2]
    save = "--save-png" in sys.argv
    try:
        main(example_dir, stem, save)
    except Exception as ex:
        print(f"{stem},error:{type(ex).__name__}:{str(ex)[:60].replace(',',';')},,,,,,,,")
        sys.exit(1)
