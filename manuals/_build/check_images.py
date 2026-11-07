from PIL import Image
from pathlib import Path
for p in sorted(Path("manuals/assets").glob("*.png")):
    im = Image.open(p)
    print(f"{p.name:28s} {im.size[0]:4d}x{im.size[1]:4d} {im.mode}")
