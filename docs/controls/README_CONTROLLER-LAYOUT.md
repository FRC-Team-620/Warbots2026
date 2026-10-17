# Controller Diagrams

Driver and operator controller diagrams generated from the keybinding code, so they always
match the robot.

- **Diagrams:** [MASTER.md](MASTER.md), always the current version. Link people here.
- **What changed:** [RELEASE_NOTES.md](RELEASE_NOTES.md)

To see the images in VS Code, open a `.md` file and press `Cmd+Shift+V` (`Ctrl+Shift+V` on
Windows). GitHub shows them inline.

## How it works

`SingleControl.java` (unified) and `DoubleControl.java` (driver + operator) say which button
runs each robot function. The generator reads them and draws the diagrams. Each change to the
bindings creates a new version, and old versions are kept.

```
docs/controls/
├── MASTER.md               GENERATED: current diagrams + history. Don't edit.
├── configs.properties      which Java class and controller picture each config uses
├── functions.properties    robot function -> label shown on the diagram
├── controller-sources/     original pictures, never edited
├── controller-layouts/     <type>/vN/: picture + button positions, no keybindings
├── diagrams/               GENERATED: <config>/vN/diagram.svg + bindings.md
└── tools/                  generator + tests
```

Keybindings appear only in `diagrams/`. A layout that contains one is rejected.

Layouts: `xbox/v1` (in use by all configs), `gamesir-tarantula-2k/v2` (drawing), and
`gamesir-tarantula-2k/v1` (photo). Open each layout's `layout.svg` to see its button map.

## Changing a keybinding

1. Edit `SingleControl.java` or `DoubleControl.java`.
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

It runs 15 checks in about 20 seconds and exits 1 on failure. The GitHub Action runs it too.

**Human,** after every new version and before events:

1. Run `./gradlew reviewControlDiagrams` and open `build/control-review/index.html`.
2. Work through the checklists on the page, including pressing every button on the real
   robot.
3. Sign off: `java docs/controls/tools/ControlDiagramGenerator.java sign-off dual --by "Name"`.

MASTER.md marks unchecked versions **not yet checked**.

## Reading a diagram

- **Colors** show the mechanism.
- **Red CONFLICT** means one button runs two things.
- **Grey *not wired*** means the button is bound in code but does nothing on the robot.
- **Not on a button** means the function has no button in this config.

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
