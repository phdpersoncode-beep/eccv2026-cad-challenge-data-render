# TODO

Ordered next steps. Method and constants: `PLAN.md`. Project rules: `CLAUDE.md`.

## 1. View-scale normalization in renders

Problem: normalization is by the longest of *all three* axes, so a flat or elongated part
draws two tiny views and one large one. The VLM then has to infer an arbitrary scale factor
that carries no information about the geometry.

- Keep the current 1:1 A4 sheet render as the **canonical** output — it is what matches the
  challenge's DXF distribution, and evaluation renders must stay in it.
- Add a `--normalize-views` mode as a *training-time* transform: compute each view's stroke
  bbox, then place each view into a fixed cell of a fixed canvas, rescaled to a constant fill
  fraction (keep aspect ratio, keep the three-view third-angle arrangement).
- Use **one shared scale for all three views** of a part (per-view scales would break the
  cross-view size relationships a reader uses to reconstruct the solid).
- Record the applied scale + per-view bboxes in the manifest so renders are invertible.
- Verify both modes on `examples/` before batch use.

## 2. Build out the pipeline (`PLAN.md` §4)

1. **`pyrender/` package** — port `prototype/` into the module layout of `PLAN.md` §2.1
   (`sheet`, `raster`, `from_dxf`, `from_step`, `from_cadquery`, `verify`) with a CLI
   `python -m pyrender dxf|step|cadquery <in> <out.png>`.
2. **`pyrender/verify.py`** — reproduce the `PLAN.md` §3 coverage table on `examples/`;
   run it in CI on a small subset so projection conventions can't silently regress.
3. **`scripts/build_dataset.py`** — multiprocessing pool, per-item subprocess timeout,
   skip-and-log, resumable manifest (id, source, status, timing, edge counts, output paths).
4. **Corpus runs** — CAD-Recode / DeepCAD-CadQuery first (they carry code labels), ABC and
   Fusion360 second (image side only, pretraining/augmentation).
5. **Optional:** emit challenge-format DXF alongside PNG (layer `0`,
   `Continuous`/`HIDDEN` linetypes via `ezdxf`).

## 3. Error taxonomy → data augmentation

We cannot close the OpenCASCADE-vs-Parasolid gap (see `docs/KNOWN_DIFFERENCES.md`). Instead,
measure it and reproduce it as augmentation so the VLM is invariant to it.

- Run `parallel_compare.py` over all **2018** pairs in `examples/` (previous numbers are from
  26 pairs only). Persist per-pair metrics + per-view stroke diff masks.
- Cluster the residuals into named failure modes and **list the example stems in each class**,
  so the augmentation can be replicated from the ground truth in training code. Seeds observed
  so far:
  - **extra blend/tangent silhouette** — OCC draws a fillet silhouette SolidWorks merges away
    (`000014`).
  - **hidden/visible misclassification on coincident edges** — the class the coincidence
    filter targets (`000001` before filtering).
  - **HLR timeout / pathological B-splines** — no render at all (`000003`, >25 min).
- Turn each class into a concrete augmentation op with a rate fitted to its measured frequency
  (e.g. randomly add/drop short silhouette strokes; flip short strokes white↔red near
  coincidences; jitter stroke width). Emit the fitted rates as a JSON spec the training code
  consumes.
- Deliverable: `docs/RESIDUAL_ERRORS.md` (classes, frequencies, example stems) +
  `config/augmentation.json`.
