#### Driver (port 0, `driver`, image `xbox/v1`)

| Input | Robot function | Group | Status |
|---|---|---|---|
| Left stick (X axis) | Drive left / right `translationX()` | Drive | ok |
| Left stick (Y axis) | Drive forward / back `translationY()` | Drive | ok |
| Left stick (press) | Slow mode (hold) `slowdown()` | Drive | ok |
| Right stick (X axis) | Rotate `rotation()` | Drive | ok |
| Right stick (press) | Hood down `hoodDown()` | Shooter | ok |
| LT | Face drive direction (hold) `faceDriveDirection()` | Drive | ok |
| RT | Auto-aim at hub (hold) `autoAim()` | Drive | ok |
| LB | Turbo (hold) `turbo()` | Drive | ok |
| RB | Reset field-forward `resetForward()` | Drive | ok |

#### Operator (port 1, `operator`, image `xbox/v1`)

| Input | Robot function | Group | Status |
|---|---|---|---|
| Left stick (press) | Spin up for hub shot (hold) `dutyCycleShoot()` | Shooter | ok |
| Right stick (press) | Run feeder + slapdown jiggle (hold) `runFeeder()` | Shooter | ok |
| LT | Indexer on (hold) `indexOn()` | Indexer | ok |
| RT | Feed + distance-adjusted shoot `feedAndShoot()` | Shooter | ok |
| LB | Slapdown up `slapdownMoveUp()` | Intake | ok |
| RB | Slapdown down + start intake `slapdownMoveDown()` | Intake | ok |
| A | Intake off `intakeOff()` | Intake | ok |
| B | Intake on `intakeOn()` | Intake | ok |
| X | Extake fuel (reverse intake + indexer) `extakeFuel()` | Intake | ok |
| Y | Field yeet: aim + hood up + spin up (hold) `fieldYeet()` | Shooter | ok |
| D-pad up | Climber up `climberUp()` | Climber | not wired in robot code |
| D-pad down | Climber down `climberDown()` | Climber | not wired in robot code |
| D-pad left | Climber extend hooks `ClimberExtendHooks()` | Climber | not wired in robot code |
| D-pad right | Climber retract hooks `ClimberRetractHooks()` | Climber | not wired in robot code |

**Not on a button in this config:** Absolute heading `rotationABS()`
