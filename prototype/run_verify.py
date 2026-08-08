"""Driver: verify a sample of DXF/STEP pairs with hard per-item timeouts.

Each item runs as its own subprocess (verify_pair.py) so a hung OCC HLR call
can be killed. Results append to a CSV; already-done stems are skipped on rerun.

Usage: python run_verify.py <examples_dir> <n_sample> <out.csv> [seed]
CSV columns: stem,status,all,visible,hidden,hid_recall,hid_precision,seconds,
             n_vis,n_hid,fit_scale,fit_offset_mm,adj_all,adj_visible,adj_hidden
"""
import glob
import os
import random
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

SP = os.path.dirname(os.path.abspath(__file__))
TIMEOUT_S = 150
WORKERS = 4


def run_one(example_dir, stem):
    try:
        p = subprocess.run(
            [sys.executable, os.path.join(SP, "verify_pair.py"), example_dir, stem],
            capture_output=True, text=True, timeout=TIMEOUT_S)
        line = p.stdout.strip()
        return line if line else f"{stem},error:empty-output,,,,,,,,"
    except subprocess.TimeoutExpired:
        return f"{stem},timeout,,,,,,{TIMEOUT_S},,"


def main(example_dir, n_sample, csv_path, seed=0):
    stems = sorted(os.path.basename(p)[:-5] for p in glob.glob(os.path.join(example_dir, "*.step"))
                   if os.path.exists(os.path.join(example_dir, os.path.basename(p)[:-5] + ".dxf")))
    random.seed(seed)
    sample = sorted(random.sample(stems, min(n_sample, len(stems))))
    done = set()
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            done = {ln.split(",")[0] for ln in f if "," in ln}
    todo = [s for s in sample if s not in done]
    print(f"{len(sample)} sampled, {len(done)} done, {len(todo)} to go", file=sys.stderr)
    with open(csv_path, "a") as out, ThreadPoolExecutor(WORKERS) as ex:
        for line in ex.map(lambda s: run_one(example_dir, s), todo):
            out.write(line + "\n")
            out.flush()
            print(line, file=sys.stderr)


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]), sys.argv[3], seed=int(sys.argv[4]) if len(sys.argv) > 4 else 0)
