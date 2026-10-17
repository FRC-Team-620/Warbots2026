#### Driver + Operator (port 0, `driver`, image `xbox/v1`)

| Input | Robot function | Group | Status |
|---|---|---|---|
| Left stick (X axis) | Drive left / right `translationX()` | Drive | ok |
| Left stick (Y axis) | Drive forward / back `translationY()` | Drive | ok |
| Left stick (press) | Spin up for hub shot (hold) `dutyCycleShoot()` | Shooter | ok |
| Right stick (X axis) | Rotate `rotation()` | Drive | **CONFLICT** |
| Right stick (X axis) | Absolute heading `rotationABS()` | Drive | **CONFLICT** |
| Right stick (press) | Hood down `hoodDown()` | Shooter | **CONFLICT** |
| Right stick (press) | Run feeder + slapdown jiggle (hold) `runFeeder()` | Shooter | **CONFLICT** |
| LT | Feed + distance-adjusted shoot `feedAndShoot()` | Shooter | ok |
| RT | Auto-aim at hub (hold) `autoAim()` | Drive | ok |
| LB | Indexer on (hold) `indexOn()` | Indexer | **CONFLICT** |
| LB | Turbo (hold) `turbo()` | Drive | **CONFLICT** |
| RB | Reset field-forward `resetForward()` | Drive | ok |
| A | Intake off `intakeOff()` | Intake | ok |
| B | Intake on `intakeOn()` | Intake | ok |
| X | Extake fuel (reverse intake + indexer) `extakeFuel()` | Intake | ok |
| D-pad up | Climber up `climberUp()` | Climber | ok |
| D-pad down | Climber down `climberDown()` | Climber | ok |
| D-pad left | Climber extend hooks `ClimberExtendHooks()` | Climber | **CONFLICT** |
| D-pad left | Slapdown down + start intake `slapdownMoveDown()` | Intake | **CONFLICT** |
| D-pad right | Climber retract hooks `ClimberRetractHooks()` | Climber | **CONFLICT** |
| D-pad right | Slapdown up `slapdownMoveUp()` | Intake | **CONFLICT** |

**Not on a button in this config:** Face drive direction (hold) `faceDriveDirection()`, Slow mode (hold) `slowdown()`, Field yeet: aim + hood up + spin up (hold) `fieldYeet()`
