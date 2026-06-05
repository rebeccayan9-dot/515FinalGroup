# SafeStep — Design Files

This folder holds the hardware design deliverables.

## In this folder
- **[`HARDWARE.md`](HARDWARE.md)** — pinout, I²C address map, haptic channel map,
  BOM, and tunable thresholds (kept in sync with `src/main.cpp`).

## ⚠️ Add the binary design-tool exports here

The text reference above is committed, but the **native CAD/PCB design files must
be added by the team** (they can't be derived from firmware). Drop the following
into this folder and commit them:

| File type | Example | Tool |
|---|---|---|
| Schematic source | `safestep.kicad_sch` / `.sch` | KiCad / Eagle |
| PCB layout source | `safestep.kicad_pcb` / `.brd` | KiCad / Eagle |
| Schematic PDF | `schematic.pdf` | (export — easy to review on GitHub) |
| Fabrication outputs | `gerbers.zip` | for board fab |
| Enclosure / mechanical | `cane_mount.step` / `.f3d` / `.stl` | Fusion 360 / etc. |
| Wiring diagram | `wiring.png` / `.svg` | Fritzing / hand-drawn |

> Tip: also export a **schematic PDF or PNG** — GitHub renders those inline, so
> reviewers (and graders) can see the design without opening a CAD tool.

If a design lives only in a cloud tool (e.g. Fusion 360, EasyEDA), add a link
here **and** commit a PDF/PNG snapshot so the repo is self-contained.
