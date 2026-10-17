# Controller Diagrams

Driver and operator controller diagrams generated from the keybinding code, so they always
match the robot.

- **Diagrams:** [MASTER_CONTROLLER-LAYOUT.md](MASTER_CONTROLLER-LAYOUT.md), always the current version. Link people here.
- **What changed:** [RELEASE_NOTES_CONTROLLER-LAYOUT.md](RELEASE_NOTES_CONTROLLER-LAYOUT.md)

To see the images in VS Code, open a `.md` file and press `Cmd+Shift+V` (`Ctrl+Shift+V` on
Windows). GitHub shows them inline.

## How it works

The diagrams are built only from the keybinding files in
`src/main/java/org/jmhsrobotics/frc2026/controlBoard/`:

- `ControlBoard.java`: the list of robot functions (no buttons)
- `DoubleControl.java`: driver and operator buttons. **This is the one the robot uses**
  (`new DoubleControl()` in `RobotContainer.java`), so the Dual Controller diagram matches the
  robot.
- `SingleControl.java`: unified controller buttons, only used if `RobotContainer.java` is
  switched to `new SingleControl()`

Edits to other robot code (including `RobotContainer.java`) never change a diagram or trigger
the GitHub Action, which keeps merge conflicts down. Each change to the bindings creates a new
version, and old versions are kept.

```
docs/controls/
├── MASTER_CONTROLLER-LAYOUT.md         GENERATED: current diagrams + history. Don't edit.
├── README_CONTROLLER-LAYOUT.md         this file
├── RELEASE_NOTES_CONTROLLER-LAYOUT.md  what changed
├── configs.properties                  which Java class and controller picture each config uses
├── functions.properties                robot function -> label shown on the diagram
├── controller-sources/                 original pictures, never edited
├── controller-layouts/                 <type>/vN/: picture + button positions, no keybindings
├── diagrams/                           GENERATED: <config>/vN/diagram.svg + bindings.md
└── tools/                              generator + tests
```

Keybindings appear only in `diagrams/`. A layout that contains one is rejected.

Layouts: `xbox/v1` (in use by all configs), `gamesir-tarantula-2k/v2` (drawing), and
`gamesir-tarantula-2k/v1` (photo). Open each layout's `layout.svg` to see its button map.

## Changing a keybinding

1. Edit `DoubleControl.java` (what the robot uses) or `SingleControl.java`.
2. For a new function, also add a label to `functions.properties`:
   `fn.myFunction=Intake|Text drivers read`.
3. Push. A GitHub Action regenerates the diagrams and commits them to your branch, so
   `git pull` afterward.

To generate locally: `./gradlew controlDiagrams -Pnote="Moved intake to B"`.

## Adding a controller picture

```bash
T=docs/controls/tools/ControlDiagramGenerator.java
java $T add-source ps5 ~/Downloads/ps5.png             # save the original
java $T add-layout ps5 ps5-cropped.png --from xbox/v1  # make a layout from it, or a cropped copy
java $T preview-layout ps5/v1                          # check: build/control-layouts/ps5-v1.svg
```

Then edit `controller-layouts/ps5/v1/anchors.properties`:

- Set `source=` to the original.
- Move each `anchor.<key>=x,y,side,name` onto its button. Read x,y off the preview grid.
- Keys are the Java method names: Xbox `a`, `leftBumper`; PS5 `cross`, `L1`; joystick
  `button3`, `pov90`.

To use it, set e.g. `dual.operator.image=ps5/v1` in `configs.properties` and generate.

A traced SVG drawing reads better than a photo. Draw it on a canvas the same size as the photo
so the button positions still line up.

## Testing

**Automated:**

```bash
./gradlew testControlDiagrams     # or: java docs/controls/tools/ControlDiagramTest.java
```

It runs 16 checks in about 20 seconds and exits 1 on failure. The GitHub Action runs it too.

**Human,** after every new version and before events:

1. Run `./gradlew reviewControlDiagrams` and open `build/control-review/index.html`.
2. Work through the checklists on the page, including pressing every button on the real
   robot.
3. Sign off: `java docs/controls/tools/ControlDiagramGenerator.java sign-off dual --by "Name"`.

The MASTER file marks unchecked versions **not yet checked**.

## Reading a diagram

- **Colors** show the mechanism.
- **Red CONFLICT** means one button runs two things.
- **Not on a button** means the function has no button in this config.
- Buttons bound in code but never used by the robot are listed on the review page, not on the
  diagram.

## Commands

| Gradle | Plain Java (`java docs/controls/tools/ControlDiagramGenerator.java ...`) | Does |
|---|---|---|
| `controlDiagrams [-Pnote=".."] [-Pforce]` | `generate [--note ".."] [--force]` | New version if anything changed |
| `checkControlDiagrams` | `check` | Fail if out of date |
| `testControlDiagrams` | `java docs/controls/tools/ControlDiagramTest.java` | Automated tests |
| `reviewControlDiagrams` | `review` | Human review page |
| | `sign-off <config> --by "Name"` | Record a human check |
| | `add-source` / `add-layout` / `preview-layout` | Controller pictures |

## Troubleshooting

| Problem | Fix |
|---|---|
| `STALE` / out of date | `./gradlew controlDiagrams` |
| `has no anchor for it` | Add that button to the layout's `anchors.properties` |
| `Dependency requires ... JVM 17` | Use WPILib's VS Code terminal, or the plain `java` commands |
| `./gradlew: Permission denied` | `chmod +x gradlew` |
