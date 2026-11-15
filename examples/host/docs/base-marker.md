# Two ways to aim the arm

A tap on the phone screen is a pixel. Turning it into a point on the table needs
the camera's pose, and there are two ways to know it.

## Mode 1 — handheld camera, tag on the base

The camera moves. The tag does not, and it is in every frame the user taps on.

1. Print `base-marker.pdf` at **100%** (check the 4 cm bar with a ruler).
2. Paste it **upright on the side of the base**, facing the operator — not on the
   table. A phone held at chest height sees it face-on, where a tag lying flat is
   seen at a glancing angle.
3. The side is the **black square, border included** — that is what the detector's
   corners span, and what PnP solves against. A 4x4 tag is 6 modules across (4 of
   payload inside one module of border each side), so a tape measure taken across
   the pattern *inside* the border is smaller: a 40 mm inner pattern is a 60 mm
   marker (`base_marker.outer_size(0.04) == 0.06`). Getting this wrong scales the
   estimated depth by the same ratio, which a test tap shows as a point tens of
   centimetres from the finger.
4. Measure the rest in the robot's base frame:
   - the tag's **centre** `x, y, z` in metres, `z` being its height above the
     table,
   - its **heading**: which way the printed face points, in **degrees** from the
     base's +x, counter-clockwise seen from above.
5. Put it in `examples/host/.env`:
   ```
   REBOT_MARKER_AT=0.000,0.047,0.043,90
   REBOT_MARKER_SIZE=0.04
   ```

   The white margin's width never enters the maths; it only has to be there (one
   module, a seventh of the square, is plenty).
6. Choose **Handheld + base tag** with the switch in the tap card.

The **Camera** chip can switch between the tag and a fitted rig at any time. The
choice travels with each plan and each marker, so neither mode needs a server
restart. With a rig fitted the page starts in fixed mode; the tag stays one click
away, and without a rig handheld is the start.

### The standard location

The RS URDF's `base_link.STL` settles most of this: a 140 x 200 x 10 mm foot plate
under a round column of radius ~47.5 mm rising to z = 75 mm, with the J1 axis
through its centre. A tag centred on the column's side, at mid-height, sits at
radius 47 mm and z = 43 mm. That is the standard placement, taped to the +y side:

```
REBOT_MARKER_AT=0.000,0.047,0.043,90
```

| the tag faces | `REBOT_MARKER_AT` |
|---|---|
| +x (the arm's forward) | `0.047,0.000,0.043,0` |
| +y (the standard) | `0.000,0.047,0.043,90` |
| -x (the back) | `-0.047,0.000,0.043,180` |
| -y | `0.000,-0.047,0.043,270` |

Errors go into the tap point one for one: 5 mm off in `x` is 5 mm off at the
finger, and a heading a half-turn wrong mirrors the point through the tag's plane
instead — which is unmistakable on the first tap.

Every phone tap then: solves the tag in the page on the tapped frame, composes
`T_base_cam` from those four numbers, and intersects the tap's ray with the
plane at the selected height — **10 cm above the worktop** by default. The
intersection moves along the ray as the height changes. (The workstation's
`/live` page is fixed-rig only and never uses the tag: it reads the constant
`T_base_cam` from `/reach.json` and runs the identical step with the phone's own
helper, so the two views cannot drift.) Tap and drag up or down on the preview to
change that height; the badge next to the zoom chip shows the current value. Each
marker keeps the height it was tapped at, so changing the badge later affects the
next marker only; the plan carries each marker's own tip point.

No calibration session, and the phone can be picked up and moved between taps.

## Mode 2 — fixed camera

The camera stays put; after calibration the tag is not needed in the frame.

### With the tag in reach: one capture, no sweep

If the fixed camera can see the tag, that still **is** the calibration. The tag
is the same one, in the same place, with the same `REBOT_MARKER_AT` as Mode 1.

1. Open the **Rig** card on `/live` (or add `?rig=1` to the URL).
2. Press **Capture one pose**.
3. The new still is scanned for the tag. When it is found, its PnP gives
   `T_cam_tag`, `T_base_tag` comes from `REBOT_MARKER_AT`, and the rig is
   written and loaded immediately — **the arm never moves**. The card shows the
   tag's size in pixels, its grazing angle and the corner reprojection rms.
4. If the tag is not in that still, nothing is written and the card says so.
   Either **Retake the still** with the tag in view, or tap the tool tip in the
   picture and run the sweep as below.

The same path is available without the page:

```bash
# one still; with the tag visible and REBOT_MARKER_AT set, this writes the rig
python app/calibrate_extrinsics.py --capture-pose --camera rtsp://...
```

`POST /capture/pose` does the same over HTTP: the response carries `marker`,
`verification` and `written` when the tag solved the camera.

The tag path is fast, but it is a single PnP rather than a fitted sweep. At a
metre a 4 cm tag is ~40 px across, so a corner error of half a pixel is about
1% of the depth — a centimetre or so at the tap. That is a usable rig, not a
metrology one; the sweep is the more accurate option, and it is still the one to
use when the camera cannot see the tag. The capture card reports the numbers it
judged (corner rms, tag size, grazing angle) so the result is not a black box.

The tag capture deliberately reads no motor registers, so it works even when the
arm gateway or the CAN bus is down. The trade-off is the mount **witness**: with
no arm pose there is nothing to mask the gripper out of the reference frame, so
that rig has no drift check. Run the sweep once afterwards if you want the
witness; the tag-written rig still plans normally.

### Without the tag: the sweep

1. Run the calibration sweep once (the panel's *Run the sweep*): the arm visits
   eight poses and the fit solves `T_base_cam` and the tool tip's offset, then the
   verification decides whether to write the rig.
2. Choose **Fixed camera** with the switch.

From then on each tap projects through that rig: ray → plane at the selected
height (10 cm above the worktop by default). Re-run the sweep if the bracket is
bumped — or, before the sweep, try **Capture one pose** again if the tag is
visible.

## What the two modes share

The tap's metric meaning is the same in both: the pixel's ray meets the table
plane (`REBOT_PLANE_Z`), and the arm goes to `REBOT_HOVER_M` above it — 10 cm by
default, adjustable per tap on the phone. The difference is only where the
camera's pose came from.

The board ends every move where it began, so the arm goes out to the point, holds,
and comes back. That is the platform's semantics, not a choice made here.

## Which to use

Handheld is the one that survives a camera that is not bolted down, at the cost of
keeping the tag in view in every tapped frame. Fixed is freer to use once it is
calibrated, but it is only as good as the last sweep, and a nudged tripod is
silent until a tap lands 5 cm off.

## What the tag buys

The corner-to-camera geometry is a plain PnP: at a metre, a 5 cm tag is ~50 px, so
half a pixel of corner noise is about 1% of depth — a centimetre. That is the
error in the *camera*, and it lands on the tap point; the arm's own repeatability
is smaller.

If the tag is pasted upside down, its printed "up" is inverted, which is a 180°
error about the face normal. The symptoms are a point reflected through the tag
plane; the fix is to set the heading 180° away from the measured one, or repaste
it.
