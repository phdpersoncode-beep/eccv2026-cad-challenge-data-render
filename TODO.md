# Open problem: OpenCASCADE draws more line than SolidWorks on blend-heavy parts

Handoff document. Read `PLAN.md` for the full method and `FIX_IDEAS.md` for the fixes already
made and the approaches already refuted. This file states the goal, what is done, and the one
substantive problem left — with the evidence gathered so far, so the next investigation starts
from measurement rather than from hypotheses.

## 1. The goal

Build a training corpus of **(technical-drawing image, CadQuery code)** pairs for the ECCV 2026
CAD challenge, without SolidWorks.

The challenge ships DXF drawings produced by a SolidWorks pipeline (`src/SolidWorksDatasetPrep.cs`)
from normalized STEP files. We are reimplementing the geometric core of that pipeline in
CadQuery/OCP (OpenCASCADE) so that drawings rendered from *arbitrary* STEP files — or from
CadQuery scripts, which is where the code labels come from — land in the **same visual
distribution** as the challenge's own drawings. If our renders differ systematically from the
challenge's, a model trained on ours sees a different input distribution at test time.

Success is therefore measured as *image agreement between the two paths*, not as CAD accuracy:

```
challenge DXF  ──► ezdxf ─────┐
                              ├──► same rasterizer ──► PNG, black bg, white = visible, red = hidden
STEP ──► normalize ──► OCC HLR ┘
```

Metric: **symmetric stroke coverage** — the fraction of one render's stroke pixels within 2 px
of the other's, worse direction, computed per class (all / visible / hidden). See `PLAN.md` §3.

## 2. Where things stand

Two renderer defects are fixed (this branch); details and evidence in `FIX_IDEAS.md`.

- **Empty views.** Exact HLR abandoned entire views when the B-rep had been rebuilt by
  `BRepTools_Modifier`. Fix: never rebuild — the view rotation rides as a `TopLoc` location and
  the normalization is applied to the 2D polylines. 4 blank views in a 288-part scan → 0.
- **Phantom silhouettes.** OCC's NURBS silhouette fit sometimes diverges, emitting a B-spline
  with poles 20–200× the part size. Fix: drop polylines leaving the part's own projected
  bounding box. Fires on 3 of 500 parts, removes at most 2 strokes, zero regressions.

Verified out-of-sample (the 2018 pairs are split by `md5(stem)`; iteration happens on a 250-stem
dev sample, scoring on a disjoint 250-stem held-out sample):

| | dev `adj_all` | dev ≥0.90 | held-out `adj_all` | held-out p10 |
|---|---|---|---|---|
| before | 0.9824 | 0.964 | 0.9754 | 0.9271 |
| after | **0.9863** | **0.976** | **0.9769** | **0.9305** |

Across the 500-part sample: 71 % ≥ 0.99, 89 % ≥ 0.95, 96 % ≥ 0.90, 99 % ≥ 0.80.

**Keep this protocol.** Every failure class here was found by staring at particular parts, so a
change tuned on them will look better than it is. Score on both splits and report both.

## 3. What is left, in three buckets

Only the first is a renderer problem.

| bucket | what it is | verdict |
|---|---|---|
| **A. Over-drawing at blends** | we emit more line than SolidWorks on fillet-heavy parts | **open — this document** |
| B. Ground-truth scale jitter | SolidWorks normalized each part with its approximate `GetPartBox`, so the challenge drawing's scale is off by a per-part factor. `005823` is drawn at **0.338×** our size (score 0.081); `004419` 0.865×; `001253` 0.947× | data property, not a bug. Mirror as training augmentation (`PLAN.md` §2.4) |
| C. Drafting conventions | `002313`: SolidWorks draws a thread as a sparse zigzag at the flanks; we draw every helix turn, filling the boss with red hatch (hidden precision 0.23) | convention difference. Could be worth a simplified-thread rule one day; not a filter |

## 4. Bucket A — the open problem

### Symptom

`003966` is the clearest case (all 0.604, visible 0.631, hidden 0.496). Structurally the drawing
is right — same part, same three views, same features — but it carries extra strokes at fillets
and blend corners. `005150` (0.870) and `007072` (0.864) are milder instances of the same thing.

### What has been established

1. **It is not one bad view.** Per-view agreement on `003966` is 0.668 / 0.574 / 0.574
   (front/top/right) — uniformly poor, not a projection or placement error.
2. **We consistently draw more line.** Stroke pixels, ours vs the DXF, per view:
   `003966` 7381/5314, 9314/5584, 6718/3448 (1.39× / 1.67× / 1.95×);
   `005150` ~1.3× in every view. The disagreement is excess, not displacement.
3. **On the milder parts it is entirely sharp edges.** Rendering each HLR compound separately
   and counting pixels the DXF has nothing near: on `005150` and `007072` *every* false positive
   comes from `VCompound`/`HCompound`; `OutLineVCompound`/`OutLineHCompound` contribute **zero**.
   On `003966` all four compounds are ~32–42 % wrong, i.e. it is further gone than the others.
4. **They are real edges, not artifacts.** The extra strokes sit 1–5 mm from the nearest
   ground-truth stroke (median 1.4 / 2.1 mm, never under 0.5 mm) — not offset duplicates. Their
   median length is 2.1 mm (`005150`) and 4.5 mm (`007072`); total 228 mm of 3242 mm and 349 mm.
5. **Almost nothing is missing.** `005150`: 1056 false-positive red px and 712 false-positive
   white px, against 39 + 1 missing. SolidWorks omits these edges outright rather than
   classifying them differently.
6. **Visually they are corner blends** — where two fillets meet, we draw a small spray of short
   strokes plus an extra arc. `python prototype/diff_map.py examples 005150 d.png 360,10,700,300`.

This matches what the challenge authors document in `docs/KNOWN_DIFFERENCES.md`: "tangent,
coplanar, coincident, internal, or occluded edges can be merged or omitted" by SolidWorks'
Parasolid kernel, and the omission is kernel-dependent rather than rule-based.

### What has been tried and refuted — do not repeat without new evidence

- **Segment-level coincidence filtering** (cut the covered stretches of a hidden edge rather than
  dropping whole edges). The partial-overlap population is real — 10–25 % of hidden length — but
  the fix regresses the composite on 17 dev parts against 1, and on the most precision-limited
  parts both precision *and* recall fall: the fragments it cuts are red the ground truth draws.
- **A near-tangent dihedral filter** (treat edges below a kink threshold as tangent, matching
  SolidWorks' "tangent edges hidden" with a looser tolerance than OCC's G1 test). The population
  is real — `005150`/`007072` carry 59/60 edges in the 0.01–1.0° band that the clean control
  `000000` lacks entirely — but as a *detector of the strokes that actually disagree* it is
  useless: at any threshold from 0.05° to 5° it catches 1 of 63 and 4 of 61 false positives while
  destroying 56 of 326 and 66 of 655 good ones. Measured end to end it lowers `007072`'s hidden
  score 0.860 → 0.826.
- **A minimum-stroke-length filter** (`FIX_IDEAS.md`'s original suggestion). The strokes are
  2–5 mm, indistinguishable from legitimate short edges.
- **An identity `BRepBuilderAPI_Transform` at import**, which re-conditions OCC's NURBS silhouette
  fits and helps this exact part (`003966` 0.604 → 0.689, stroke count 803 → 1267): a wash across
  the dev sample and runtime-neutral, but it reintroduces the empty-view failure. Worth revisiting
  only together with a fix for that.

The lesson from all four: **any filter must be scored as a detector before it is implemented** —
how many of the disagreeing strokes does it catch, and how much legitimate geometry does it
destroy? A filter that cannot reach ~0.8 precision at useful recall will cost more than it saves.

### Threads worth pulling next

- **Identify the source edge.** Nobody has yet matched a false-positive stroke back to the input
  edge that produced it. Project every edge of the solid per view and match by proximity, then
  compare the false-positive sources against the legitimate ones *within the same part*: adjacent
  surface types, how the dihedral angle varies **along** the edge (a corner-patch boundary may be
  tangent for part of its length — only the median was measured), the size of the faces it bounds,
  and whether it belongs to a chain of tangent edges. This is the most direct unexplored route.
- **Why is `003966` different in kind?** It is the only part where the silhouette compounds are
  also ~32 % wrong. Establish whether that is the same mechanism amplified or a second one before
  treating it as the representative case — the milder parts may be the better subject.
- **Ask what SolidWorks actually did.** `src/SolidWorksDatasetPrep.cs` is in this repo. It sets
  the display mode and tangent-edge option explicitly; check whether it also sets an edge-quality
  or simplification option whose effect we have not reproduced. Cheaper than reverse-engineering
  Parasolid from the DXFs.
- **Decide whether it is worth fixing at all.** At the distribution level this costs ~0.01 of mean
  agreement. `PLAN.md` §2.4 argues the remaining gap is absorbed by SFT + RL. Quantify the benefit
  before spending more: the honest answer may be that bucket B (scale-jitter augmentation) matters
  more to downstream training than bucket A does.

## 5. Reproducing

```bash
pip install cadquery ezdxf numpy scipy pillow

# one pair: metrics + renders
python prototype/verify_pair.py examples 003966 --save-png

# where it disagrees, per pixel (magenta/yellow = we invent, cyan/green = we miss)
python prototype/diff_map.py examples 003966 diff.png

# a corpus sample, resumable
python prototype/run_verify.py examples 300 results.csv
```

CSV columns: `stem,status,all,visible,hidden,hid_recall,hid_precision,seconds,n_vis,n_hid,fit_scale,fit_offset_mm,adj_all,adj_visible,adj_hidden`.
`hid_precision` is the "too much red" direction, `hid_recall` the "too little red" one.

**Caveat:** the `seconds` column starts its clock inside `verify_pair.main()`, which then imports
cadquery/OCP — so it includes ~2 s of import and cannot be compared across harnesses that import
at different points. Time `step_polylines()` directly for anything performance-related, and not
while a parallel sweep is running. Two wrong runtime conclusions were drawn from this column.

`step_polylines(step_file, report)` fills `report` with `dropped_bodies`, `empty_views`,
`poly_fallback_views` and `offbbox_edges` — a corpus run should flag any part where these are
non-trivial rather than shipping the drawing.
