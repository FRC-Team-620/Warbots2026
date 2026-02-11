package org.jmhsrobotics.frc2026.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

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

  // private final LEDPattern hubSwitchingPattern =
  //     LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.5));

  public LEDToControlMode(LED led) {
    this.led = led;

    addRequirements(led);
  }

  // TODO: Based on decided factors, change LED pattern
  @Override
  public void execute() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {

      if ((time - 40 == 28) || (time - 40 == 53) || (time - 40 == 78) || (time - 30 == 103)) {
        new LEDFlashPattern(led, LEDPattern.solid(Color.kWhite), LEDPattern.solid(Color.kGreen));
      }
      if (((ally.get() == Alliance.Blue) && (GameState.getHubStatus() == Hub.ACTIVE))
          || ((ally.get() == Alliance.Red) && (GameState.getHubStatus() == Hub.INACTIVE)))
        led.setPattern(blueHubActivePattern);
      else if (((ally.get() == Alliance.Red) && (GameState.getHubStatus() == Hub.ACTIVE))
          || ((ally.get() == Alliance.Blue) && (GameState.getHubStatus() == Hub.INACTIVE)))
        led.setPattern(redHubActivePattern);
      else led.setPattern(scrollingRainbow);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
