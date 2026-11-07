import re, pathlib
files = [pathlib.Path("readme.md")] + list(pathlib.Path("manuals").glob("*.md"))
missing = []
for md in files:
    text = md.read_text(encoding="utf-8")
    for m in re.finditer(r"\[[^\]]*\]\(([^)]+)\)", text):
        target = m.group(1).strip()
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        target = target.split("#")[0]
        p = (md.parent / target).resolve()
        if not p.exists():
            missing.append((str(md), target))
print("checked", len(files), "markdown files")
print("missing links:", missing if missing else "none")
