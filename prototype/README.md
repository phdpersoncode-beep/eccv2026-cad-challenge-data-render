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

# compare all pairs (4 processes, 25 min/item timeout), then build a side-by-side sheet
EXAMPLES_DIR=/path/to/examples python /path/to/prototype/parallel_compare.py
python /path/to/prototype/contact_sheet.py sheet.png 000000 000004 000014
```

The scripts import each other (they share one rasterizer); invoking them by full path works
because Python puts the script's own directory on `sys.path`. Outputs (`cmp_*.png`,
`result_*.txt`, sheets) are written to the current working directory.
