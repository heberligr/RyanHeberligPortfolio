# segslicer: "3+2" segmented slicer for a DIY 5-axis FDM printer

**Status: Milestone 1 (project skeleton and math core).** The full README
(pivot calibration, tuning, first-print workflow) comes in Milestone 7.

## Setup

```bash
cd five_axis_slicer
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Python 3.11 or newer.

## Run

```bash
python -m segslicer path/to/model.stl
python -m segslicer --machine my_machine.yaml   # use a different machine file
```

You can also type a path into the **STL path** box and click **Load**.

## Run the tests

```bash
python -m pytest
```

## Configure your machine

Edit `config/machine.yaml`. Every value marked **FILL IN** is a placeholder.
The most important ones are:

| Key | What it is |
|---|---|
| `kinematics.pivot` | Machine XYZ where the A and C axes cross |
| `kinematics.a_axis.sign` / `c_axis.sign` | `1` if a positive G-code angle turns the table by the right-hand rule, `-1` if the other way |
| `kinematics.*.min_deg/max_deg` | Rotary travel limits, in commanded degrees |
| `table.surface_z` | Machine Z of the table top at A = 0 |
| `hotend.*` | Collision envelope (cone at the tip and a cylinder above it) |

`config/settings.yaml` holds the PrusaSlicer path (used from Milestone 4).

## Code map

```
segslicer/
  config.py      machine.yaml / settings.yaml -> dataclasses, with validation
  transforms.py  rotations: direction -> +Z and back, rotate about a point
  kinematics.py  A/C inverse kinematics, part frame <-> machine XYZ
  collision.py   hotend cone + cylinder model, sampled-surface collision check
  mesh_io.py     STL loading, placing the part on the table
  viewer.py      Polyscope window and ImGui panel
tests/           pytest unit tests
```

### The kinematics in one picture

The part sits on the C table, and the C table sits on the A trunnion. So a
point `p` on the part (part frame = machine frame when A = C = 0) ends up at

```
machine = R_A(a) · R_C(c) · (p − pivot) + pivot
```

with `R_A` a rotation about X and `R_C` a rotation about Z. For a segment with
build direction `d`, the inverse kinematics (IK) finds `a` and `c` so that
`R_A R_C d = +Z`. That makes the segment's layers horizontal under the nozzle:

1. Turn C so `d` lies in the machine's YZ plane: `c = 90° − atan2(dy, dx)`.
2. Tilt A by the angle between `d` and vertical: `a = atan2(√(dx²+dy²), dz)`.

There is always a mirror solution (`a → −a`, `c → c − 180°`). The code tries
both, keeps those inside the axis limits, and picks the one with the smallest
C move from the previous pose.

### The collision check

Obstacle meshes (the table and segments already printed) are sampled into
surface points about `collision.sample_spacing` mm apart. For a given nozzle
tip and tool direction, the points are moved into the tool frame and tested
against the cone/cylinder envelope. The envelope is grown slightly so a
surface cannot slip between samples, which makes the check err on the safe
side. Points within `hotend.tip_ignore_height` of the tip along the axis are
ignored, so the surface being printed on does not count as a hit.

## Milestone 1 manual test checklist

1. `python -m segslicer some_part.stl`: the part sits on the grey table,
   centred on the orange pivot point.
2. Open **Hotend / IK test**. The green hotend sits at the top centre of the part.
3. Drag **Tilt** to about 45° and change **Azimuth**. The A/C numbers update.
   When the hotend hits the part or table, it turns red and the panel names
   the obstacle.
4. Tick **Machine view**. The part and table rotate by A/C, and the hotend
   should always point straight down (vertical) at the tilted part.
5. Set Tilt above 90°. The panel should say the direction is unreachable.
