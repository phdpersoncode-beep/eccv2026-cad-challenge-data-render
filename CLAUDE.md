# CLAUDE.md

## What this repo is

Upstream (`README.md`, `src/*.cs`, `scripts/*.ps1`, `config/`, `docs/`) is the ECCV 2026 CAD
Workshop challenge authors' **SolidWorks** dataset-prep pipeline: STEP → normalized STEP +
third-angle A4 drawing (DXF/PDF/SVG) + 3D renders. Windows + SolidWorks only. **Do not modify
upstream files** — they are the reference specification.

**Our goal:** reproduce the DXF/drawing half of that pipeline with **CadQuery / OCP
(OpenCASCADE)** on Linux, so we can turn public 3D corpora (CAD-Recode, DeepCAD, ABC,
Fusion360) into *(technical-drawing PNG, CadQuery code)* pairs for VLM training. The challenge
provides no training data.

Design docs: `PLAN.md` (architecture + measured verification results — read it first),
`TODO.md` (**the current task list — always check it for next steps**).

## Layout

| path | what |
|---|---|
| `PLAN.md` | reverse-engineered SolidWorks spec + our architecture + verification numbers |
| `TODO.md` | next steps |
| `prototype/` | **our** validated proof-of-concept (dxf_render, step_render, compare, parallel_compare, contact_sheet) |
| `examples/` | 2018 DXF/STEP pairs (ground truth); the first 26 also have `.svg` + `test_outputs/` (manifests, projection maps) |
| `src/`, `scripts/`, `config/`, `docs/` | upstream SolidWorks pipeline — reference only |

`config/challenge_2026.json` and `examples/test_outputs/projection_maps/*.json` are the
authoritative source for every constant below.

## Pipeline invariants (verified against all 26 original examples — do not change casually)

- **Normalize:** center bbox at origin, uniform scale so longest axis = **100 mm** (drawing
  model). Published `examples/*.step` are pre-normalized to **1.8 mm** — scale by `100/1.8`.
  No rotation.
- **Sheet:** A4 landscape 297 × 210 mm, third angle, scale 1:1.
- **Views** (model origin lands at a fixed sheet position; identical for every part):
  | view | center (mm) | screen mapping | depth axis |
  |---|---|---|---|
  | front | (83, 57) | (x, y) | +z |
  | top | (83, 158) | (x, −z) | +y |
  | right | (223, 57) | (−z, y) | +x |
- **DXF reading rule:** modelspace layer `0` only; linetype `Continuous` → visible,
  `HIDDEN` → hidden; skip `INSERT` (center marks are annotation on layer `10`).
- **HLR:** rotate the *solid* per view, always project from +Z with one
  `HLRAlgo_Projector`. visible = `VCompound + OutLineVCompound`,
  hidden = `HCompound + OutLineHCompound`; `Rg1Line*`/`IsoLine*` excluded (= SolidWorks
  "tangent edges hidden"). Discretize at 0.05 mm.
- **Coincidence filter (essential):** OCC reports a hidden edge that projects onto a visible
  edge; SolidWorks merges it. Resample hidden polylines at 0.1 mm, KD-tree query against
  visible samples, drop the edge if ≥90 % of samples are within 0.15 mm. Without it hidden-line
  precision collapses (`000001`: 0.45 → 0.97).
- **Render:** black background, **white** = visible, **red** = hidden, nothing else. 4 px/mm →
  1188 × 840. Draw hidden first, visible second. Flip y (DXF is y-up).

## Verification

Metric: symmetric stroke coverage at 2 px tolerance, per class, worse direction.
Current result on 25/26 original pairs (`000003` exceeded a 25 min HLR budget):
all 0.997 mean / 0.966 min, visible 0.995 / 0.951, hidden 0.991 / 0.957 — **zero fitted
parameters**. Any change to the invariants above must be re-verified with
`prototype/parallel_compare.py`; a regression in these numbers means the change is wrong.

## Running things

Deps are **not installed** in this container: `pip install cadquery ezdxf numpy scipy pillow`
(Python ≥ 3.10; `cadquery` ships OCP). No SolidWorks, no Windows anywhere in our path.

```bash
cd <scratch dir>                       # scripts write cmp_*.png / result_*.txt to cwd
python prototype/dxf_render.py  examples/000000.dxf  out_dxf.png
python prototype/step_render.py examples/000000.step out_step.png
EXAMPLES_DIR=$PWD/examples python prototype/parallel_compare.py     # all pairs, 4 procs
python prototype/contact_sheet.py sheet.png 000000 000004 000014    # eyeball side by side
```

Prototype scripts import each other and rely on the script dir being on `sys.path`.

## Conventions

- Simplicity and measured accuracy over cleverness; every constant traceable to a projection
  map or config file, not tuned by hand.
- HLR has a heavy tail (minutes on spline-rich parts) — anything batch runs in a subprocess
  pool with a per-item timeout and a skip-and-log manifest.
- Residual DXF-vs-STEP mismatch is a *kernel* difference (Parasolid vs OpenCASCADE on
  tangent/blend silhouettes), acknowledged upstream in `docs/KNOWN_DIFFERENCES.md`. We do not
  chase it below the documented floor — we catalogue it and turn it into training augmentation
  (see `TODO.md`).
- Branch for this work: `claude/cad-rendering-documentation-k6862y`.
