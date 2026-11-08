from PIL import Image
from pathlib import Path
for p in sorted(Path("manuals/assets").glob("*.png")):
    im=Image.open(p); print(p.name, im.size)
