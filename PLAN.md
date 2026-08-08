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

1. **Import & normalize.** `cq.importers.importStep`; translate bbox center to origin, scale so
   the longest axis is 100 mm. (Challenge STEPs at 1.8 mm and raw corpus STEPs at any size both
   land in the same place.)
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

Optional (phase 2): emit true DXF files alongside PNGs — write the HLR curves as layer-`0`
entities with `Continuous`/`HIDDEN` linetypes via `ezdxf`, mirroring the SolidWorks encoding,
so the dataset also contains challenge-format DXFs, not just rasters.

---

## 3. Verification (done as part of this plan — prototype results)

Prototypes of both paths (`from_dxf`, `from_step`) were run on the 26 DXF/STEP pairs in
`examples/` (scripts in `prototype/`; side-by-side sheets can be regenerated with
`prototype/parallel_compare.py` + `prototype/contact_sheet.py`, see `prototype/README.md`).
Metric: symmetric stroke coverage — the fraction of one render's stroke pixels within 2 px
(0.5 mm) of the other's, taking the worse direction, computed per class.

Measured on 25/26 pairs (`000003` exceeded a 25-minute exact-HLR budget — see risks):

| metric | mean | median | min |
|---|---|---|---|
| all strokes | 0.997 | 1.000 | 0.966 |
| visible (white) | 0.995 | 1.000 | 0.951 |
| hidden (red) | 0.991 | 0.999 | 0.957 |

18 of 25 pairs score ≥ 0.99 on every metric with **zero fitted parameters** — the view layout,
projection directions, scale and placement reproduce SolidWorks exactly.

- The coincidence filter (§2.2.5) is what makes hidden lines match: without it, hidden-line
  precision drops below 0.5 on some parts (e.g. `000001`: 0.45 → 0.97 with the filter).
- Residual differences are the kernel-level edge-classification discrepancies the challenge
  authors themselves document in `docs/KNOWN_DIFFERENCES.md` — Parasolid and OpenCASCADE
  occasionally disagree on tangent/fillet silhouettes (e.g. `000014`, where OCC draws a blend
  silhouette SolidWorks merges away). They affect a few short strokes per part, not the
  drawing's structure.

The comparison harness becomes `pyrender/verify.py` and runs in CI on the bundled examples so
any regression in the projection conventions is caught immediately.

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
| HLR slow / hangs on pathological B-splines (observed: `000003` > 25 min) | per-item subprocess timeout; skip + manifest reason; parallel workers; optional fallback to mesh-based `HLRBRep_PolyAlgo` for the heavy tail |
| kernel classification differs from Parasolid on tangent/coincident edges | accepted — documented by challenge authors; quantified at ~2–6 % of stroke pixels on examples |
| multi-solid / assembly STEPs | skip (same policy as the SolidWorks pipeline) |
| CadQuery scripts with side effects / infinite loops | sandboxed subprocess execution, resource limits |
| a corpus part that degenerates at 100 mm scale (zero-thickness) | validate solid volume > 0 before HLR; skip otherwise |
| views overlapping sheet bounds for extreme aspect ratios | outline check; parts are ≤ 100 mm in every axis, fixed layout fits A4 as in the original pipeline; clip + flag if exceeded |
