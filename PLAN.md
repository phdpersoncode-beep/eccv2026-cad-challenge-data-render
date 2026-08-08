# Plan: Open-Source Technical Drawing Renders for VLM Training

**Goal:** build a training dataset of *(technical-drawing PNG, CadQuery code)* pairs for the
ECCV 2026 CAD challenge, without SolidWorks. We reimplement the geometric core of this
repository's SolidWorks pipeline with CadQuery / OCP (OpenCASCADE), so that renders produced
from arbitrary STEP files or CadQuery scripts match the distribution of the challenge's
DXF drawings.

Render format (both for challenge DXFs and for our own STEP-derived drawings):

- black background
- **white** strokes = visible edges
- **red** strokes = hidden edges
- nothing else (no dashes, no center marks, no border, no text)

Color already encodes the visible/hidden distinction, so hidden edges are drawn as solid red
strokes instead of dashed black ones. This keeps the raster trivially comparable between the
DXF path and the STEP path and gives the VLM a cleaner signal than dash patterns.

---

## 1. What the SolidWorks pipeline actually does (reverse-engineered)

Everything below was extracted from `src/SolidWorksDatasetPrep.cs`, `config/challenge_2026.json`,
the projection maps in `examples/test_outputs/projection_maps/`, and direct inspection of the
26 DXF/STEP/SVG triplets in `examples/`.

### 1.1 Normalization

```
center = (bbox_min + bbox_max) / 2
scale  = target / max(size_x, size_y, size_z)
p'     = scale * (p - center)          # no rotation, uniform scale
```

Two normalized copies exist per part:

| artifact                     | longest axis | used for |
|------------------------------|--------------|----------|
| drawing model                | 100 mm       | the DXF/PDF/SVG drawing, plotted 1:1 on A4 |
| published `*.step` (challenge input, `examples/*.step`) | 1.8 mm | the file participants receive |

`examples/*.step` are already normalized (verified: bbox = 1.8 mm longest axis, centered at
origin). Scaling them by `100 / 1.8` reproduces the drawing model exactly.

### 1.2 Sheet and view layout

- A4 landscape sheet: **297 × 210 mm**, DXF written in mm ( `$EXTMAX = (297, 210)` )
- third-angle projection, three views, drawing scale 1:1
- the model origin (= bbox center) of each view lands at a **fixed sheet position**:

| view  | sheet position of model origin (mm) | screen mapping (model → sheet) | depth axis (towards viewer) |
|-------|--------------------------------------|-------------------------------|------------------------------|
| front | (83, 57)                             | (x, y)                        | +z |
| top   | (83, 158)                            | (x, −z)                       | +y |
| right | (223, 57)                            | (−z, y)                       | +x |

These mappings come from the `model_to_view_transform` matrices in the projection maps
(SolidWorks row-vector convention, `sheet = p · R + t`). They are identical for every example
— view placement is deterministic and does not depend on the part.

### 1.3 Edge classification and DXF encoding

- display mode: Hidden Lines Visible (`swHIDDEN_GREYED`), **tangent edges hidden**
- all drawing geometry is written to DXF **layer `0`** as `LINE / ARC / CIRCLE / ELLIPSE /
  SPLINE / LWPOLYLINE` entities
- visible edges: entity linetype `Continuous`; hidden edges: entity linetype `HIDDEN`
  (verified consistent across all 26 examples)
- center marks are `SW_CENTERMARKSYMBOL_*` block INSERTs on layer `10` — annotation, not part
  geometry; **we ignore them**
- the other template layers (Chinese GB template names, `SLD-0`, `5`, `9`, `Defpoints`) carry
  no part geometry in any example

So a DXF → PNG renderer needs exactly one rule: *layer `0` entity, linetype `Continuous` →
white; linetype `HIDDEN` → red; INSERTs skipped.*

---

## 2. Architecture

Two render paths that share one rasterizer, plus a dataset builder:

```
                       ┌────────────────────┐
 challenge DXF ───────►│ dxf_polylines()    │──┐
                       │ (ezdxf, flatten)   │  │   shared polyline format:
                       └────────────────────┘  │   [(pts_mm, 'visible'|'hidden'), ...]
                                               ├──► rasterize() ──► PNG (1188×840)
 STEP file ──► normalize ──► 3 × OCC HLR ──────┤        black bg, white/red strokes
 (or CadQuery script ──► solid)                │
                       └── coincidence filter ─┘
```

### 2.1 Module layout (new top-level package `pyrender/`)

| file | responsibility |
|------|----------------|
| `pyrender/sheet.py` | constants: sheet size, view rotations + centers, px/mm, line widths, colors |
| `pyrender/raster.py` | polylines (mm, sheet coords) → PNG; hidden drawn first, visible on top |
| `pyrender/from_dxf.py` | DXF → polylines via `ezdxf` (`path.make_path(e).flattening(0.05)`) |
| `pyrender/from_step.py` | STEP/solid → normalize → per-view HLR → polylines |
| `pyrender/from_cadquery.py` | execute CadQuery script (subprocess sandbox, timeout) → solid → `from_step` core |
| `pyrender/verify.py` | DXF-vs-STEP render comparison harness (metrics below) |
| `scripts/build_dataset.py` | walk a corpus, parallel workers, per-item timeout, manifest CSV, resume |

Python ≥ 3.10, dependencies: `cadquery` (ships OCP), `ezdxf`, `numpy`, `scipy`, `pillow`.
No SolidWorks, no Windows, fully parallelizable on Linux.

### 2.2 STEP path — the core algorithm

1. **Import & normalize.** `cq.importers.importStep`, then collect **solids only** into a
   compound (some corpus STEPs carry stray shells/faces that poison the bounding box — one
   even reports a 2·10¹⁰⁰ bbox; fall back to shells when a file has no solid). Compute the
   **exact** bounding box with `BRepBndLib.AddOptimal` (tessellation-independent), reject
   void/degenerate boxes, translate the center to the origin and scale the longest axis to
   100 mm. Multi-body parts are kept whole. (Challenge STEPs at 1.8 mm and raw corpus STEPs
   at any size both land in the same place.)
2. **Per view, rotate the solid instead of moving the camera.** Apply the view rotation
   (transpose of the row-vector matrices in §1.2) with `gp_Trsf`, then always project with the
   same projector `HLRAlgo_Projector(gp_Ax2(origin, gp_Dir(0,0,1)))` — viewer on +Z. This
   guarantees the screen/depth conventions match the table above with no per-view sign traps.
3. **Hidden line removal.** `HLRBRep_Algo` → `Update()` → `Hide()` → `HLRBRep_HLRToShape`:
   - visible = `VCompound` + `OutLineVCompound` (sharp edges + silhouettes)
   - hidden = `HCompound` + `OutLineHCompound`
   - `Rg1Line*` / `IsoLine*` compounds are **excluded** — this is exactly SolidWorks
     "tangent edges hidden"
4. **Discretize** every edge with `GCPnts_QuasiUniformDeflection` (0.05 mm sagitta — well below
   one pixel at 4 px/mm).
5. **Coincidence filter (the one non-obvious step).** SolidWorks merges a hidden edge that
   projects exactly onto a visible edge into the visible one; OCC reports both. Without
   filtering, renders grow red fringes along white lines that the ground-truth DXFs do not
   have (measured hidden-line precision as low as 0.45). Fix: resample hidden polylines at
   0.1 mm, query a KD-tree of visible-polyline samples, and drop a hidden edge when ≥ 90 % of
   its samples lie within 0.15 mm of visible geometry. HLR already splits edges at visibility
   changes, so whole-edge dropping is safe.
6. **Place views:** `sheet = view_center + screen_xy`, rasterize.

### 2.3 Rasterization

- 4 px/mm → **1188 × 840** PNG (A4 aspect); configurable
- line width 2 px for both classes (SolidWorks weights 0.25 / 0.18 mm would be ~1 px and
  brittle for a VLM; color carries the class signal) — draw hidden first, visible second so
  crossings resolve the way drawings do
- y axis flipped (DXF is y-up, image is y-down)

### 2.4 CadQuery path (training pairs)

For corpora that provide CadQuery code (e.g. CAD-Recode's 1 M scripts, DeepCAD converted to
CadQuery):

1. run each script in a subprocess (`timeout`, memory cap, no network) and export the resulting
   solid to a B-rep shape
2. reuse §2.2 steps 1–6 → PNG
3. emit `(png, cadquery_source)` training pair; the code is the label, the render is the input
4. skip scripts that fail, produce non-solids, or produce multi-solid results (the SolidWorks
   pipeline likewise skips assemblies), recording the reason in the manifest

For STEP-only corpora (ABC, Fusion360) the same renderer produces the image side; those are
useful for pretraining/augmentation only, since there is no code label.

**Recommended augmentation (mirrors the challenge distribution, §3.2):** with some
probability, apply a per-part uniform scale jitter (×0.92–1.00) and a small 3D offset
(±4 mm at drawing scale) before projecting. The challenge drawings carry exactly this
jitter from SolidWorks' approximate bounding-box normalization, so training with it closes
most of the measurable input-distribution gap for free.

**Training strategy:** SFT on our (render, CadQuery) pairs — accepting the small remaining
distribution shift — then RL against the challenge training corpus, where rollouts are scored
by executing generated CadQuery and comparing geometry. Residual render differences (bbox
jitter, kernel edge classification) get absorbed there rather than by chasing pixel parity
with SolidWorks rule-by-rule.

Optional (phase 2): emit true DXF files alongside PNGs — write the HLR curves as layer-`0`
entities with `Continuous`/`HIDDEN` linetypes via `ezdxf`, mirroring the SolidWorks encoding,
so the dataset also contains challenge-format DXFs, not just rasters.

---

## 3. Verification (done as part of this plan — prototype results)

Metric everywhere below: **symmetric stroke coverage** — the fraction of one render's stroke
pixels within 2 px (0.5 mm) of the other's, taking the worse direction, computed per class.
What we are verifying is exactly what the VLM will see: that the DXF→image and
STEP→drawing→image paths produce consistent images. Tooling: `prototype/` (see its README);
the harness becomes `pyrender/verify.py` and runs in CI on the bundled examples so any
regression in the projection conventions is caught immediately.

### 3.1 First release — 26 curated pairs

Measured on 25/26 pairs (`000003` exceeded a 25-minute exact-HLR budget — see risks):

| metric | mean | median | min |
|---|---|---|---|
| all strokes | 0.997 | 1.000 | 0.966 |
| visible (white) | 0.995 | 1.000 | 0.951 |
| hidden (red) | 0.991 | 0.999 | 0.957 |

18 of 25 pairs score ≥ 0.99 on every metric with **zero fitted parameters** — the view layout,
projection directions, scale and placement reproduce SolidWorks exactly. Side-by-side renders
for five representative parts: `assets/plan_verification_examples.png`.

- The coincidence filter (§2.2.5) is what makes hidden lines match: without it, hidden-line
  precision drops below 0.5 on some parts (e.g. `000001`: 0.45 → 0.97 with the filter).
- Residual differences are the kernel-level edge-classification discrepancies the challenge
  authors themselves document in `docs/KNOWN_DIFFERENCES.md` — Parasolid and OpenCASCADE
  occasionally disagree on tangent/fillet silhouettes (e.g. `000014`, where OCC draws a blend
  silhouette SolidWorks merges away). They affect a few short strokes per part, not the
  drawing's structure.

### 3.2 Full corpus — all 2018 DXF/STEP pairs in `examples/`

2015 of 2018 pairs scored with exact HLR; the 3 remaining are exact-HLR timeouts
(spline-heavy parts — `000003` doesn't finish in 15 min). The mesh-based fallback
(`prototype/step_render_poly.py`, `HLRBRep_PolyAlgo` on a 0.05 mm triangulation) renders
each of them in **under 1 s**, scoring 0.93 / 0.93 / 0.49 — so the heavy tail is fully
covered: exact HLR first, poly HLR on timeout. "Aligned" = after the per-pair similarity
fit described below.

| metric | raw mean | raw median | raw p10 | aligned mean | aligned median | aligned p10 |
|---|---|---|---|---|---|---|
| all strokes | 0.943 | 1.000 | 0.812 | 0.978 | 1.000 | 0.938 |
| visible | 0.936 | 0.999 | 0.795 | 0.976 | 1.000 | 0.933 |
| hidden | 0.906 | 0.974 | 0.703 | 0.940 | 0.978 | 0.838 |

Share of pairs with all-stroke agreement ≥ 0.99 / 0.95 / 0.90 / 0.80:
raw 0.63 / 0.79 / 0.86 / 0.90 — aligned 0.69 / 0.87 / **0.94** / 0.98.
Fitted jitter: ~two-thirds of parts need none at all; among the 697 parts with a non-zero
offset the median is 0.2 mm (p95 3.5 mm), and the p5 scale is 0.986. Runtime: median 2.9 s,
p90 3.9 s, p99 6.3 s per part (three exact-HLR views) — roughly 1 000 parts/CPU-hour,
embarrassingly parallel. Review sheets for the worst-aligned pairs, a random sample of clean
pairs, and the import-hardening cases: `assets/verification_review/`.

The curated examples hid a phenomenon the full corpus exposes: **the challenge drawings carry
per-part normalization jitter.** SolidWorks normalized each part with its approximate
`GetPartBox` (documented as approximate — see `docs/KNOWN_DIFFERENCES.md`), so a drawing's
effective scale can be off by up to ~8 % and its center by several mm versus exact-bbox
normalization — rigidly, per part. To separate that jitter from real geometric disagreement,
the harness also fits a single uniform scale + 3D offset per pair (two parameters of the SW
bbox error, no rotation, nothing shape-specific) and re-scores. Examples: `006464` scores
0.59 raw but **1.000** aligned (7.5 % scale + 3.7 mm offset); `002197` 0.63 → **1.000**
(pure 3.6 mm offset).

What the tail contains (each case checked visually):

1. **SW bbox jitter** (dominant) — rigid shift/scale, geometry identical. We mirror it as
   augmentation (§2.4) and let SFT + RL absorb the rest; reproducing SolidWorks' proprietary
   bbox per part is not possible by rule, and per our training strategy it does not need to be.
2. **Kernel edge-classification differences** (as in §3.1) — a few short strokes per part.
3. **Challenge-data idiosyncrasies reproduced faithfully** — e.g. `004395`'s DXF differs from
   its STEP in the challenge data itself; whatever the drawing set contains is by definition
   the train/test distribution, so these count as distribution facts, not render errors. True
   outliers (e.g. `000540`, whose DXF shows a different part than its STEP contains) are rare
   and detectable by low aligned agreement. For training data generated from CadQuery code
   such mismatches cannot occur — image and label come from the same solid.

The importer hardening (§2.2.1) came out of this run: multi-body compounds, stray shells with
unbounded surfaces, and off-nominal STEP sizes (one at 0.59 mm instead of 1.8 mm) all appear
in the corpus and are now handled.

## 4. Implementation steps

1. **`pyrender` package** — port the validated prototypes into the module layout of §2.1 with
   a small CLI (`python -m pyrender dxf|step|cadquery <in> <out.png>`). *(~1 day)*
2. **Verification harness** — `verify.py` reproducing the §3 table on `examples/`; add a
   side-by-side contact-sheet generator for eyeballing. *(~0.5 day)*
3. **Dataset builder** — `scripts/build_dataset.py`: multiprocessing pool, per-item timeout
   (HLR on spline-heavy parts can take minutes — observed on `000003`), skip-and-log policy,
   resumable manifest (id, source, status, timings, edge counts, output paths). *(~1 day)*
4. **Corpus runs** — CAD-Recode / DeepCAD-CadQuery first (code labels), ABC second (images
   only). Budget compute: HLR averages seconds per simple part but has a heavy tail; the
   timeout + skip policy keeps throughput predictable. *(compute-bound)*
5. **Phase 2 (optional)** — DXF emission (§2.4), configurable augmentations (line width,
   resolution), isometric 3D render styles to mirror the challenge's other input modality.

## 5. Risks & mitigations

| risk | mitigation |
|---|---|
| HLR slow / hangs on pathological B-splines (3 of 2018 corpus parts; corpus p99 is only 6.3 s) | per-item subprocess timeout, then the **validated** mesh-based `HLRBRep_PolyAlgo` fallback (`prototype/step_render_poly.py`): < 1 s on all three pathological parts |
| kernel classification differs from Parasolid on tangent/coincident edges | accepted — documented by challenge authors; a few short strokes per part on examples |
| challenge drawings carry SW approximate-bbox scale/offset jitter (§3.2) | accepted as target-distribution fact; mirrored as train-time augmentation (§2.4); SFT tolerates the shift, RL on the challenge corpus closes the rest |
| multi-solid parts and STEPs with stray shells / unbounded surfaces | solids-only compound + exact `AddOptimal` bbox + validity guards (§2.2.1); genuine assemblies skipped like the SolidWorks pipeline |
| mispaired DXF/STEP items in the challenge corpus (e.g. `000540`) | detectable by low aligned agreement; irrelevant for generated training pairs (image and label share one solid) |
| CadQuery scripts with side effects / infinite loops | sandboxed subprocess execution, resource limits |
| a corpus part that degenerates at 100 mm scale (zero-thickness) | validate solid volume > 0 before HLR; skip otherwise |
| views overlapping sheet bounds for extreme aspect ratios | outline check; parts are ≤ 100 mm in every axis, fixed layout fits A4 as in the original pipeline; clip + flag if exceeded |
