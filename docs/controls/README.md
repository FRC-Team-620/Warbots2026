# Controller Diagrams

**Purpose:** Driver/operator controller diagrams generated from the keybinding code, so the
picture always matches what the robot actually does.

**Published diagrams: [MASTER.md](MASTER.md)** (current version of every config + history)

## How it flows

```
ControlBoard.java              robot functions (intakeOn, feedAndShoot, ...)
  ├─ SingleControl.java        unified config   -> which button runs each function
  └─ DoubleControl.java        driver + operator config
RobotContainer.java            what each function does (used to flag "not wired")
        │
        ▼  ControlDiagramGenerator (runs automatically on push, or by hand)
diagrams/<config>/vN/          new version only when bindings actually change
        │
        ▼
MASTER.md                      the one published page
```

## Folder layout

```
docs/controls/
├── MASTER.md                    GENERATED: published diagrams + version history. Don't edit.
├── configs.properties           which ControlBoard class -> which config, and which image each controller uses
├── functions.properties         robot function -> label + color group shown on the diagram
├── controller-images/           base controller pictures, versioned per controller type
│   └── <type>/vN/
│       ├── controller.svg|png|jpg
│       └── anchors.properties   where each button is on the picture
├── diagrams/                    GENERATED: one folder per config, one subfolder per version
│   └── <config>/vN/
│       ├── diagram.svg
│       ├── bindings.md          the same bindings as a table
│       └── version.properties   date, commit, note
└── tools/ControlDiagramGenerator.java
```

## Common tasks

**Changed a keybinding:** edit `SingleControl.java` / `DoubleControl.java` and push. The
*Controller Diagrams* GitHub Action makes a new version and updates `MASTER.md`. To run it
yourself:

```bash
./gradlew controlDiagrams -Pnote="Moved intake to B"   # -Pforce for a new version anyway
./gradlew checkControlDiagrams                          # fails if diagrams are out of date
```

No JDK 17? Plain `java docs/controls/tools/ControlDiagramGenerator.java generate --note "..."`
works with any JDK 16+.

**Added a function to ControlBoard:** add a `fn.<method>=<group>|<label>` line to
`functions.properties` (it still works without one, but the diagram shows the raw method name).

**Saving a new controller picture (new controller type, or a better image):**

```bash
java docs/controls/tools/ControlDiagramGenerator.java add-image ps5 ~/Downloads/ps5.png --from xbox/v1
```

This copies it to `controller-images/ps5/v1/` (next run: `v2`), reads its size, and copies the
anchor list to start from. Then:

1. Open `anchors.properties` and move each `anchor.<key>=x,y,side,name` onto its button
   (pixel coordinates in the picture; `side` = which label column).
2. Anchor keys are the controller method names the Java calls, so they change with the
   controller class: Xbox `a`, `leftBumper`; PS5 `cross`, `L1`; `CommandJoystick`
   `button3`, `pov90`. Use `input.<call>=<anchor>,<detail>` when several calls share one spot
   (stick X/Y axes and stick press).
3. Point a controller at it in `configs.properties`, e.g. `dual.operator.image=ps5/v1`, and
   generate. Driver and operator can use different controller types.

Only save images you have the rights to use (your own photos, or ones with a free license).

**Adding another config** (e.g. a test layout): create the `ControlBoard` implementation, then
add it to `configs=` in `configs.properties` with its `.title`, `.source`, `.controllers`, and
per-controller `.port`, `.role`, `.image`.

## Reading a diagram

- Colors = function group (Drive, Shooter, Intake, ...).
- **Red / CONFLICT** = two wired functions on the same input of the same controller.
- *Grey italic (not wired)* = bound to a button but never used outside the controlBoard
  package (e.g. the removed climber).
- "Not on a button" = in `ControlBoard` but this config returns `null` / throws for it.
