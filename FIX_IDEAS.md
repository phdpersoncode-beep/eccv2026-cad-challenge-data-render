# Worst STEP-derived renders: causes, fixes, and what the evidence refuted

Three error classes were identified from `assets/verification_review/worst_aligned_*.png`.
This file records the original hypotheses and what measurement actually showed. Two of the
three are fixed; the third turned out to be misdiagnosed and is documented as a known
residual rather than patched.

Verification protocol for every claim below: the 2018 `examples/` pairs are split by
`md5(stem)` into a dev pool and a held-out pool, and each change is scored on a 250-stem
sample of each. Dev is where the iteration happens; held-out is only ever scored, so a
threshold tuned to specific parts shows up there as a regression. Per-part figures quoted
from the review sheets are diagnostic only — none of the nine parts that motivated these
fixes is in either sample.

| | dev `adj_all` | dev >= 0.90 | held-out `adj_all` | held-out p10 |
|---|---|---|---|---|
| before | 0.9824 | 0.964 | 0.9754 | 0.9271 |
| after problems 1 + 2 | **0.9863** | **0.976** | **0.9769** | **0.9305** |

---

## 1. Missing views — FIXED

**Original hypotheses, all three wrong:** a swallowed exception in `hlr_view()`, a body
dropped by the finite-bbox filter, or `GCPnts_QuasiUniformDeflection` returning
`IsDone()==False`. Instrumenting `001306` disproved each: no getter ever raised, no body was
dropped (1 solid in, 1 kept, 40 faces, `BRepCheck_Analyzer` valid), and no edge failed to
discretize on the views that did work.

**Actual cause.** Exact HLR abandons the *entire* view — all four compounds come back
`IsNull` — when it chokes on a single face, and a face only becomes unchewable after the
B-rep has been rebuilt by `BRepTools_Modifier`. The pipeline rebuilt every shape twice:
`normalized_shape()` baked in the centering and scaling, and `rotate_for_view()` used
`BRepBuilderAPI_Transform(..., copy=True)`.

Evidence on `001306`: HLR on the raw imported solid gives `V=59 OV=2 H=17`; the same solid
after a bare `BRepBuilderAPI_Copy` — poles, knots, degrees and tolerances bit-identical —
gives nothing. Bisecting its 40 faces isolates one `Geom_BSplineSurface`: drop that face and
the rebuilt shape yields 84 edges, splice the original face back into the rebuilt set and it
yields 92. Tilting the projector by 1e-4 rad also revives it, so the fault is a numerical
degeneracy in OCC's silhouette computation at exactly the view direction that the rebuild
tips over the edge.

**Fix** (`prototype/step_render.py`): never rebuild the B-rep. `normalization()` returns the
untouched shape plus the centering/scaling as numbers, `rotate_for_view()` carries the
rotation as a `TopLoc` location, and the normalization is applied to the 2D polylines —
valid because an orthographic projection commutes with a uniform scale about the origin and
with a translation. Plus the diagnostics that should have caught this: raising getters and
dropped bodies are logged, `discretize()` falls back to uniform parameter sampling instead of
dropping an edge, and a view that comes out empty is retried on the mesh HLR engine, then
recorded in `report['empty_views']` so a corpus run can flag an incomplete drawing instead of
emitting it silently.

**Result.** Empty views on the 288-stem scan: 4 before, 0 after. `002570` 0.808 -> 0.964,
`005929` 0.780 -> 0.953, `006455` 0.648 -> 0.889, `005150` 0.636 -> 0.870.

**Cost.** HLR now runs at the STEP's own scale (every challenge STEP is 1.8 mm), 55x closer
to OCC's absolute tolerances than 100 mm, which costs 1.33x on dev and 1.46x on held-out;
99.7 % of it is `Hide()`. Investigated and rejected: `HLRAlgo_Projector`'s `gp_Trsf`
constructor does accept a scale factor but HLR computes in the unscaled space
(`Transformation().ScaleFactor()` is 1.0 while `FullTransformation()` carries the 55.6), and
a scaling `TopLoc_Location` is accepted and *is* fast but returns curves whose trim ranges
are not rescaled — silently truncated edges that an edge-count check would miss. There is no
way to hand HLR scaled geometry and get correct trims back; only a real rebuild does that,
and that is the bug above. `HLRBRep_Data::Tolerance` is hard-coded to 1e-5 at both scales and
setting it changes neither time nor output.

## 2. Phantom lines — FIXED

**Original hypothesis, wrong:** that the phantoms are short slivers, removable with a ~1 mm
minimum-length filter. Measured, the phantom strokes run 17-5243 mm, median ~200 mm. A
length filter removes none of them.

**Actual cause.** The long off-sheet strokes and the lens/"ear" shapes are one bug, not two.
Both come only from `OutLineVCompound`/`OutLineHCompound`, never from sharp edges. OCC fits
each NURBS face's silhouette with a single-span degree-8..10 B-spline, and when that fit does
not converge the poles land 20-200x the part size away — on `003966`, a degree-10, 11-pole
curve with poles at 182 units on a 1.8 mm part — and the curve genuinely evaluates out there.
The parametrization is normal and `GCPnts` reports success, so nothing downstream notices.
Running HLR face by face over `005929`'s 597 faces, exactly one produces an outline that
leaves its own bbox, and it is the B-spline; the 325 cylinders, 110 planes, 108 tori, 36
cones and 2 spheres never misbehave.

**Fix** (`prototype/step_render.py`): nothing on the solid can project outside the solid's own
projected bounding box, so drop any polyline that leaves it. The box is computed analytically
per view (the view rotations are signed axis permutations, so it is exact — verified to 1e-7
mm against `BRepBndLib.AddOptimal`), applied before the coincidence filter so a phantom
visible stroke cannot mask a genuine hidden edge, and counted in `report['offbbox_edges']`.

**Why the margin is not a tuned parameter.** Across 10 471 strokes on 16 parts the two
populations are separated by 700x: every legitimate stroke is within 0.001 mm of the box,
every phantom at least 0.698 mm outside. Margins of 0.02 / 0.1 / 0.5 mm give identical
renders. On sharp edges the filter is a measured no-op (0 offenders in 9192 strokes).

**Result.** `005929` 0.953 -> 0.984, `003966` 0.413 -> 0.604, and `005929`'s fitted similarity
offset collapses 5.35 mm -> 0.00, so the phantom had been poisoning the alignment fit too.
This is a tail fix: on the 500 random dev+held-out parts it changes 2 and regresses **none**,
which is the property that matters — it does not touch legitimate geometry.

## 3. Hidden-line disagreement — MISDIAGNOSED, not fixed

**Original hypothesis:** the coincidence filter drops a hidden edge only when >= 90 % of the
*whole edge* lies within 0.15 mm of visible geometry, so an edge that coincides for part of
its length survives entirely and paints red fringes alongside white. Proposed remedies were
segment-level filtering, a larger epsilon, and a rasterization backstop.

The premise is half true and the conclusion is wrong.

**The partial-overlap population is real** — 10.4 % of hidden line length on `003966`, 14.9 %
on `005150`, 25.0 % on `002313` sits strictly between "uncovered" and "fully covered".

**But segment-level filtering makes the drawing worse.** Implemented and swept over the dev
sample: hidden-line median improves (0.9772 -> 0.9849) while the composite regresses on 17
parts against 1, and the p10 tail drops. On the two most precision-limited parts both
precision *and* recall fell, meaning the fragments it cut were red the ground truth does
draw. Reverted.

**Because the fringes are not where the fix assumed.** Rendering the per-pixel disagreement
shows the extra strokes are not alongside white lines at all — they cluster at fillets and
rounded corners, in both colours at once (`005150`: 1056 false-positive red px, 712
false-positive white px, but only 39 + 1 missing). Attributing those pixels to their source
compound: on `005150` and `007072` **every** false positive comes from the sharp-edge
compounds and silhouettes contribute exactly zero. The strokes sit 1-5 mm from the nearest
ground-truth stroke (median 1.4 / 2.1 mm, never under 0.5 mm), so they are not offset
duplicates — they are real edges of the solid that SolidWorks does not draw, at a corner
blend where two fillets meet.

**A tangency filter cannot find them either.** SolidWorks ran with tangent edges hidden and
OCC's G1 test is tighter than Parasolid's, so near-tangent edges arriving as sharp edges was
a natural suspect. The dihedral census supports the premise — `005150` and `007072` carry 59
and 60 edges in the 0.01-1.0 deg band that the clean control `000000` does not have at all —
but as a *detector of the actual bad strokes* it is useless: at any threshold from 0.05 to
5 deg it catches 1 of 63 and 4 of 61 false positives while dropping 56 of 326 and 66 of 655
good ones (precision 0.02-0.06). Relaxing the coverage bar to 50 % catches 33 of 63 at the
cost of 186 of 326 good strokes. Implemented and measured end to end, it lowers `007072`'s
hidden score 0.860 -> 0.826, exactly as the detector predicts.

**Conclusion.** This residual is the kernel difference the challenge authors document in
`docs/KNOWN_DIFFERENCES.md` — "tangent, coplanar, coincident, internal, or occluded edges can
be merged or omitted" — arising at blend corners, and it is not reproducible by a rule over
OCC's output. It is bounded: 228 mm of 3242 mm of stroke on `005150`, 349 mm on `007072`,
and across the 500-part dev+held-out sample "too much red" is the binding constraint on 30-38 %
of parts against 10-12 % for "too little red". Per PLAN.md §3.2 it is a property of the target
distribution, to be absorbed by SFT + RL rather than chased rule by rule.

The rasterization backstop was not implemented: the extra red is 1-5 mm from any ground-truth
stroke, so redrawing visible strokes wider cannot cover it, and since the rasterizer is shared
with the DXF path it would change every visible stroke in the dataset.

---

## Verification loop

```bash
# single pair, with renders
python prototype/verify_pair.py examples 005150 --save-png
# corpus sample, resumable CSV
python prototype/run_verify.py examples 300 results.csv
```

Score any change on both a dev and a held-out sample and report both. A change that only
moves the parts you were looking at has not been verified.
