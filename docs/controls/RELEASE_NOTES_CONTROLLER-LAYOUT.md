# Controller Diagrams: Release Notes

## 1.1 (2026-10-17)

- Diagrams are now built only from the keybinding files in `controlBoard/`. Other robot code,
  including `RobotContainer.java`, no longer changes a diagram or triggers the GitHub Action.
  This avoids merge conflicts.
- Buttons bound in code but never used by the robot moved from the diagrams to the review
  page, under "Bound but unused".
- The docs were renamed to `*_CONTROLLER-LAYOUT.md`.
- Both diagrams are now v2.

**Needs a decision**

- The robot uses `DoubleControl.java`, so the **Dual Controller** diagram is the one that
  matches the robot.
- Unified controller conflicts:
  - LB runs both Turbo and Indexer.
  - Right-stick press runs both Hood down and Run feeder.
  - D-pad left and right each run a climber function and a slapdown function.
  - Right stick X runs both Rotate and Absolute heading.
- The removed climber is still bound to the D-pad in both setups. Deleting it clears the
  D-pad conflicts.
- Neither diagram has been checked by a person yet.

## 1.0 (2026-10-17)

Controller diagrams are now generated from the robot code, so they always match what the robot
does. See [MASTER_CONTROLLER-LAYOUT.md](MASTER_CONTROLLER-LAYOUT.md) for the diagrams and [README_CONTROLLER-LAYOUT.md](README_CONTROLLER-LAYOUT.md) for how to use them.

**New**

- Diagrams for the unified (one controller) and dual (driver + operator) setups. They flag
  conflicts and buttons that do nothing.
- A GitHub Action regenerates the diagrams whenever keybindings change. Every change is a new
  version, and the old ones are kept.
- Controller pictures are swappable: Xbox and the GameSir Tarantula 2K are included, and other
  controllers can be added.
- Automated tests (`./gradlew testControlDiagrams`) and a human review page with sign-off
  (`./gradlew reviewControlDiagrams`).
