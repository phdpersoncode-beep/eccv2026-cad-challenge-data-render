# Fix ideas for the worst STEP-derived renders

Three error classes seen in `assets/verification_review/worst_aligned_*.png`, with likely
cause and concrete fix. Ordered by expected payoff. (Note: some worst pairs are corpus data
issues — `000540` is a DXF/STEP mispair — and cannot be fixed by rendering.)

## 1. Missing views

**Cause candidates (in `prototype/step_render.py`):**
- `hlr_view()` wraps each compound getter in `try/except: continue` — an HLR failure for one
  view is silently swallowed, leaving that view empty.
- Bodies dropped by the finite-bbox filter can delete an entire small part.
- `GCPnts_QuasiUniformDeflection` returning `IsDone()==False` silently skips edges.

**Fixes:**
- Per-view sanity check: if a view yields zero visible edges while the part has faces, retry
  that view with the poly-HLR fallback (`step_render_poly.py`); if still empty, log + flag
  the part in the manifest instead of emitting a silently wrong render.
- Log every swallowed exception and every dropped body (stderr + manifest column).
- For `IsDone()==False`, fall back to uniform parameter sampling of the curve.

## 2. Phantom tangent lines at arcs/fillets

**Cause:** OCC's `OutLineVCompound` includes silhouettes of blend/fillet surfaces that
SolidWorks (Parasolid) merges into tangent edges and hides (`000014` shows a mild case). At
grazing view angles these show up as lines "out of nowhere" tangent to arcs.

**Fixes (in order of simplicity):**
- Drop outline edges shorter than a threshold (e.g. 1 mm at 100 mm model scale) — most
  phantoms are short slivers.
- Coincidence-style filter for **visible-vs-visible**: drop an outline edge if ≥90 % of its
  samples lie within ε of *sharp* (VCompound) edges — silhouette duplicating a real edge.
- Curvature test: sample the underlying face along the outline edge; if the surface normal
  stays within ~1° of perpendicular to the view direction over the whole edge (grazing
  tangency, not a true silhouette crossing), drop it. More faithful, more code.
- If phantom persists: check whether it comes from the poly fallback (mesh silhouettes are
  noisier) — raise mesh quality (0.01 mm deflection) or restrict fallback to timeout parts.

## 3. Hidden lines blending with the visible lines that should cover them

**Cause:** the coincidence filter drops a hidden edge only when ≥90 % of the *whole edge* is
within 0.15 mm of visible geometry. A hidden edge that coincides for part of its length (or
is offset ~1 px by SW bbox jitter) is kept entirely, drawing red fringes alongside white.

**Fixes:**
- **Segment-level filtering** (main fix): resample each hidden polyline at 0.1 mm and drop
  covered *segments* instead of whole edges; emit the surviving sub-polylines. Removes the
  partial-overlap fringes without deleting genuinely hidden portions.
- Raise ε from 0.15 mm to ~0.3 mm (≈ 1.2 px) to absorb rasterization/jitter offsets; keep
  the 90 % rule per segment run, not per edge.
- Rasterization backstop: after drawing, re-draw visible strokes once more with width+1 px —
  guarantees white covers red at exact coincidences regardless of geometry filtering.

## Verification loop for any of the above

`python prototype/run_verify.py examples 300 results.csv` (seeded sample, resumable CSV,
~15 min on 4 cores) before/after each change; regenerate the review sheets with
`verify_pair.py --save-png` + `contact_sheet.py` for the same worst stems to eyeball.
Success criterion: aligned p10 up from 0.938 without median regression.
