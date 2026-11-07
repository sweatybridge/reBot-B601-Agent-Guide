from pathlib import Path
for md in Path("manuals").glob("*.md"):
    lines = md.read_text(encoding="utf-8").splitlines()
    in_table = False
    counts = []
    issues = []
    for i, line in enumerate(lines, 1):
        s = line.strip()
        if s.startswith("|") and s.endswith("|"):
            c = s.count("|")
            if not in_table:
                in_table = True; counts = []
            counts.append((i, c, s))
        else:
            if in_table:
                if len(set(c for _, c, _ in counts)) > 1:
                    issues.append((counts[0][0], [c for _, c, _ in counts]))
                in_table = False
    if in_table and len(set(c for _, c, _ in counts)) > 1:
        issues.append((counts[0][0], [c for _, c, _ in counts]))
    if issues:
        print(md.name, "TABLE ISSUES", issues)
    else:
        print(md.name, "tables ok")
