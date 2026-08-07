"""Parallel DXF-vs-STEP comparison with per-item timeout; writes one result line per stem."""
import glob
import os
import sys
import multiprocessing as mp

SP = os.getcwd()  # result_*.txt and cmp_*.png are written to the current directory
EX = os.environ.get("EXAMPLES_DIR", os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "examples"))
TIMEOUT_S = 1500


def one(stem):
    import numpy as np
    from dxf_render import dxf_polylines, render
    from step_render import step_polylines
    from compare import masks, sym_coverage, cov

    dimg = render(list(dxf_polylines(os.path.join(EX, stem + ".dxf"))),
                  os.path.join(SP, f"cmp_dxf_{stem}.png"))
    simg = render(step_polylines(os.path.join(EX, stem + ".step")),
                  os.path.join(SP, f"cmp_step_{stem}.png"))
    dw, dr = masks(dimg)
    sw, sr = masks(simg)
    line = (f"{stem}: all={sym_coverage(dw | dr, sw | sr):.3f} "
            f"visible={sym_coverage(dw, sw):.3f} hidden={sym_coverage(dr, sr):.3f} "
            f"| hid recall={cov(dr, sr):.3f} precision={cov(sr, dr):.3f}")
    with open(os.path.join(SP, f"result_{stem}.txt"), "w") as f:
        f.write(line + "\n")
    return line


def worker(stem):
    try:
        return one(stem)
    except Exception as ex:
        line = f"{stem}: ERROR {ex}"
        with open(os.path.join(SP, f"result_{stem}.txt"), "w") as f:
            f.write(line + "\n")
        return line


if __name__ == "__main__":
    stems = sys.argv[1:] or sorted(
        os.path.basename(p)[:-5] for p in glob.glob(os.path.join(EX, "0000*.step")))
    todo = [s for s in stems if not os.path.exists(os.path.join(SP, f"result_{s}.txt"))]
    print(f"{len(todo)} stems to process: {todo}")
    ctx = mp.get_context("spawn")
    with ctx.Pool(4) as pool:
        results = [(s, pool.apply_async(worker, (s,))) for s in todo]
        for s, r in results:
            try:
                print(r.get(timeout=TIMEOUT_S))
            except mp.TimeoutError:
                print(f"{s}: TIMEOUT (> {TIMEOUT_S}s)")
                with open(os.path.join(SP, f"result_{s}.txt"), "w") as f:
                    f.write(f"{s}: TIMEOUT\n")
