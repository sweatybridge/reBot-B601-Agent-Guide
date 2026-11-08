# Image pipeline

Every image in `manuals/assets/` is generated from code in this directory — there
are no stock photos, external assets, or browser dependencies. The renderer is
Pillow at 2× supersampling, with a small flat-shaded perspective renderer for the
close-up cards.

## Regenerate

From the repository root, with Pillow available in `.rebot-venv`:

```powershell
.\.rebot-venv\Scripts\python.exe manuals\_build\make_cards.py
.\.rebot-venv\Scripts\python.exe manuals\_build\make_layouts.py
.\.rebot-venv\Scripts\python.exe manuals\_build\make_camera_detail.py
.\.rebot-venv\Scripts\python.exe manuals\_build\make_atlas.py
```

If Pillow is missing:

```powershell
$env:UV_CACHE_DIR = "$PWD\.uv-cache"
uv pip install --python .rebot-venv\Scripts\python.exe --cache-dir $env:UV_CACHE_DIR pillow
```

## Files

| File | Job |
|---|---|
| `draw.py` | Canvas, fonts, palette, dashed/arrow/dimension helpers |
| `scene3d.py` | Perspective camera, painter's-algorithm faces, flat shading |
| `props.py` | Kitchen props: counter, sink, hob, board, grinder, tools |
| `scenes_b.py`, `scenes_c.py` | The five scenes for layouts B and C |
| `card.py` | The 1680 × 1120 mess-card layout |
| `plans.py` | Plan and section primitives, including the hood, camera and view rectangle |
| `make_cards.py` | Builds the 15 mess cards |
| `make_layouts.py` | Builds the 3 layout sheets |
| `make_camera_detail.py` | Builds `assets/hood-camera-detail.png` |
| `make_atlas.py` | Builds `assets/mess-atlas.png` |
| `workspace_probe.py` | Samples the tool-down cleaning envelope (pure stdlib) |
| `reach_bands.py` | Reach by height and wall-approach bands |
| `check_links.py` | Verifies every image and relative link resolves |

## Envelope numbers

`workspace_probe.py` and `reach_bands.py` sample joint space with no numpy and
report the radii quoted in the manuals. The design uses:

- **600 mm** working radius for assigned tasks,
- **755 mm** as the reserve band from the vendor reach,
- roughly **600 mm** of useful vertical wall reach above the counter.

Run them if the chain or the joint limits change:

```powershell
.\.rebot-venv\Scripts\python.exe manuals\_build\workspace_probe.py
.\.rebot-venv\Scripts\python.exe manuals\_build\reach_bands.py
```

## Drawing conventions

- Plans are drawn in millimetres at a fixed pixels-per-mm scale; each sheet has a
  1 m scale bar. The section is schematic, not a second scaled view.
- The 600 mm disc is solid; the 755 mm maximum is dashed. The 40° dead wedge is
  hatched and points into the wall/corner/soffit.
- Mess cards are close-ups, not orthographic projections. The arm enters from the
  side that matches the layout (right for A, left for B, above for C).
- Text in the cards is intentionally sparse; the manual carries the full method,
  the size window, and the stop condition.
- The optional hood camera is the small module icon at the hood edge; the dashed
  rectangle is its useful view on the worktop, not the full optical footprint.

## Editing dimensions

If a base position or appliance size changes, update it in **both** places:

1. `make_layouts.py` (plan coordinates and notes), and
2. the corresponding `manuals/layout-*.md` tables and prose.

The dimension annotations on the sheet are the source of truth for the plan; the
manual repeats the same numbers for the reader.

