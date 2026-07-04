---
status: done
---

# TestGUI should open wider so the playfield view is bigger

## Description

The Robot Test GUI currently opens at 1200x700
(`host/robot_radio/testgui/__main__.py:369`, `window.resize(1200, 700)`)
with the horizontal splitter initialized to `[420, 780]`
(`host/robot_radio/testgui/__main__.py:989`). At that size the playfield
(SIM MODE) view on the right is cramped.

The stakeholder resized the window manually to roughly **1920x1110**
logical points (screenshot provided in the request) and wants that to be
the default opening geometry. All of the extra width should go to the
right-hand playfield/canvas pane, not the left controls column:

- Left controls column keeps its current/natural width (~900 px in the
  reference screenshot, which accommodates the Sim Errors three-column
  panel from sprint 075).
- The playfield image pane gets the remaining width and the stretch
  priority, so the camera/sim view renders as large as possible.

## Acceptance sketch

- On launch, the window opens at (or near) 1920x1110 — ideally clamped
  to the available screen geometry so it still fits on smaller displays.
- The initial splitter sizes give the growth to the right pane (e.g.
  left stays ~900, right takes the rest), and resizing the window wider
  continues to grow the playfield pane, not the controls column.
- Existing minimum canvas size (`canvas.py:360`, 400x280) and layout
  tests still pass.
