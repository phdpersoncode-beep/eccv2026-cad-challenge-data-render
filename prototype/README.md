# Verification prototypes for PLAN.md

Proof-of-concept scripts that validate the CadQuery/OCP reimplementation of the SolidWorks
drawing pipeline against the DXF/STEP pairs in `../examples/`. See `../PLAN.md` for the
method and the measured results.

Requirements: `pip install cadquery ezdxf numpy scipy pillow` (Python ≥ 3.10, no SolidWorks).

```bash
cd <some scratch dir>

# render a challenge DXF to PNG (black bg, white visible, red hidden)
python /path/to/prototype/dxf_render.py examples/000000.dxf out_dxf.png

# render the paired STEP the same way via OCC hidden line removal
python /path/to/prototype/step_render.py examples/000000.step out_step.png

# mesh-based HLR fallback for parts where exact HLR is pathologically slow
python /path/to/prototype/step_render_poly.py examples/000003.step out_step.png

# corpus-scale verification: sample N pairs, subprocess-per-item with hard timeout,
# raw + similarity-aligned metrics appended to a resumable CSV
python /path/to/prototype/run_verify.py /path/to/examples 300 results.csv

# single pair with saved renders (cmp_dxf_*.png / cmp_step_*.png in cwd)
python /path/to/prototype/verify_pair.py /path/to/examples 000000 --save-png

# side-by-side sheet from saved renders
python /path/to/prototype/contact_sheet.py sheet.png 000000 000004 000014
```

`verify_pair.py` reports each pair twice: raw agreement, and agreement after fitting a
per-part uniform scale + 3D offset (`fit_scale`, `fit_offset_mm` in the CSV). The fit
isolates the SolidWorks approximate-bounding-box normalization jitter present in the
challenge drawings from real structural differences — see PLAN.md §3.

The scripts import each other (they share one rasterizer); invoking them by full path works
because Python puts the script's own directory on `sys.path`. Outputs (`cmp_*.png`,
`result_*.txt`, sheets) are written to the current working directory.
