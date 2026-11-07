from PIL import Image, ImageDraw, ImageFont
from pathlib import Path
root = Path("manuals/assets")
cards = [(f"{l}-{i:02d}", root / f"layout-{l.lower()}-mess-{i:02d}.png") for l in "ABC" for i in range(1,6)]
tw, th = 420, 280
W, H = 5*tw, 3*th
sheet = Image.new("RGB", (W, H), (247,243,234))
d = ImageDraw.Draw(sheet)
font = ImageFont.truetype(r"C:\Windows\Fonts\consolab.ttf", 14)
for idx, (label, path) in enumerate(cards):
    im = Image.open(path).convert("RGB").resize((tw, th), Image.LANCZOS)
    x, y = (idx % 5)*tw, (idx//5)*th
    sheet.paste(im, (x, y))
    d.rectangle([x, y, x+tw-1, y+th-1], outline=(210,200,185))
    d.text((x+8, y+6), label, font=font, fill=(20,30,28))
sheet.save(root.parent / "assets" / "mess-atlas.png")
print("saved")

