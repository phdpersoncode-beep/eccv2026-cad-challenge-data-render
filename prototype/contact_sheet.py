"""Build a contact sheet: rows = examples, cols = (DXF render, STEP render)."""
import glob
import os
import sys
from PIL import Image, ImageDraw

SP = os.getcwd()  # reads cmp_*.png from the current directory (see compare.py)


def main(stems, out, scale=0.5):
    pairs = []
    for stem in stems:
        d, s = os.path.join(SP, f"cmp_dxf_{stem}.png"), os.path.join(SP, f"cmp_step_{stem}.png")
        if os.path.exists(d) and os.path.exists(s):
            pairs.append((stem, Image.open(d), Image.open(s)))
    if not pairs:
        print("nothing to do")
        return
    w, h = pairs[0][1].size
    sw, sh = int(w * scale), int(h * scale)
    pad, label_h = 8, 22
    sheet = Image.new("RGB", (sw * 2 + pad * 3, (sh + label_h + pad) * len(pairs) + pad), (25, 25, 25))
    draw = ImageDraw.Draw(sheet)
    y = pad
    for stem, dimg, simg in pairs:
        draw.text((pad, y + 2), f"{stem}  DXF (ground truth)", fill=(180, 180, 180))
        draw.text((sw + pad * 2, y + 2), f"{stem}  STEP via OCC HLR", fill=(180, 180, 180))
        y += label_h
        sheet.paste(dimg.resize((sw, sh)), (pad, y))
        sheet.paste(simg.resize((sw, sh)), (sw + pad * 2, y))
        y += sh + pad
    sheet.save(out)
    print(f"wrote {out} ({len(pairs)} pairs)")


if __name__ == "__main__":
    stems = sys.argv[2:] or sorted(
        os.path.basename(p)[8:-4] for p in glob.glob(os.path.join(SP, "cmp_dxf_*.png")))
    main(stems, sys.argv[1])
