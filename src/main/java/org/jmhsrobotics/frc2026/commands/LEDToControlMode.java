package org.jmhsrobotics.frc2026.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import org.jmhsrobotics.frc2026.Constants;
// import org.jmhsrobotics.frc2026.subsystems.intake.Intake;
import org.jmhsrobotics.frc2026.subsystems.led.LED;
import org.jmhsrobotics.frc2026.util.GameState;
import org.jmhsrobotics.frc2026.util.GameState.Hub;

public class LEDToControlMode extends Command {
  private LED led;

  // used for transition flashes

  double time = DriverStation.getMatchTime();
  //   private Intake intake;

  private final LEDPattern searchModePattern = LEDPattern.rainbow(255, 255);
  private final LEDPattern scrollingRainbow =
      searchModePattern.scrollAtAbsoluteSpeed(
          MetersPerSecond.of(1), Constants.LEDConstants.kSpacing);

  // TODO: Decide on what factors determine what color the LEDs turn
  private final LEDPattern blueHubActivePattern = LEDPattern.solid(Color.kBlue);
  private final LEDPattern redHubActivePattern = LEDPattern.solid(Color.kRed);
  private final LEDPattern bothHubsActivePattern = LEDPattern.solid(Color.kPurple);

  // private final LEDPattern blueToRedLedPattern = blueHubActivePattern.blend(redHubActivePattern);

  private final LEDPattern blueHubSwitchingPattern =
      LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.1));

  private final LEDPattern redHubSwitchingPattern =
      LEDPattern.solid(Color.kRed).blink(Seconds.of(0.1));

  private final LEDPattern transitionSwitchingPattern =
      LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.1));

  public LEDToControlMode(LED led) {
    this.led = led;

    addRequirements(led);
  }

  @Override
  public void execute() {
    time = DriverStation.getMatchTime();
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent() && DriverStation.isTeleop()) {

      if ((time < 133) && (time > 130)) {
        led.setPattern(transitionSwitchingPattern);
      } else if (((time < 33) && (time > 30))
          || ((time < 57) && (time > 55))
          || ((time < 83) && (time > 80))
          || ((time < 107) && (time > 105))) {
        if (((ally.get() == Alliance.Red) && (GameState.getHubStatus() == Hub.ACTIVE))
            || ((ally.get() == Alliance.Blue) && (GameState.getHubStatus() == Hub.INACTIVE))) {
          led.setPattern(redHubSwitchingPattern);
        } else if (((ally.get() == Alliance.Blue) && (GameState.getHubStatus() == Hub.ACTIVE))
            || ((ally.get() == Alliance.Red) && (GameState.getHubStatus() == Hub.INACTIVE))) {
          led.setPattern(blueHubSwitchingPattern);
        }
      } else if ((time > 133) || (time <= 30)) {
        led.setPattern(bothHubsActivePattern);
      } else if (((ally.get() == Alliance.Blue) && (GameState.getHubStatus() == Hub.ACTIVE))
          || ((ally.get() == Alliance.Red) && (GameState.getHubStatus() == Hub.INACTIVE)))
        led.setPattern(blueHubActivePattern);
      else if (((ally.get() == Alliance.Red) && (GameState.getHubStatus() == Hub.ACTIVE))
          || ((ally.get() == Alliance.Blue) && (GameState.getHubStatus() == Hub.INACTIVE)))
        led.setPattern(redHubActivePattern);
      else led.setPattern(scrollingRainbow);
    } else {
      led.setPattern(scrollingRainbow);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
