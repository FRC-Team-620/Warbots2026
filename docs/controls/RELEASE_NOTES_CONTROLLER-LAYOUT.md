# Controller Diagrams: Release Notes

## 1.0 (2026-10-17)

Controller diagrams are now generated from the robot code, so they always match what the robot
does. See [MASTER.md](MASTER.md) for the diagrams and [README.md](README.md) for how to use them.

**New**

- Diagrams for the unified (one controller) and dual (driver + operator) setups. They flag
  conflicts and buttons that do nothing.
- A GitHub Action regenerates the diagrams whenever keybindings change. Every change is a new
  version, and the old ones are kept.
- Controller pictures are swappable: Xbox and the GameSir Tarantula 2K are included, and other
  controllers can be added.
- Automated tests (`./gradlew testControlDiagrams`) and a human review page with sign-off
  (`./gradlew reviewControlDiagrams`).

**Needs a decision**

- Unified controller: LB runs both Turbo and Indexer, and right-stick press runs both Hood down
  and Run feeder.
- The removed climber is still bound to the D-pad in both setups.
- Neither diagram has been checked by a person yet.
