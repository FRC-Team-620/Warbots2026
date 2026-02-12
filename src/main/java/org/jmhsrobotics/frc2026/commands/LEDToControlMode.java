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

  // private final LEDPattern blueToRedLedPattern = blueHubActivePattern.blend(redHubActivePattern);

  private final LEDPattern hubSwitchingPattern =
      LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.1));

  public LEDToControlMode(LED led) {
    this.led = led;

    addRequirements(led);
  }

  @Override
  public void execute() {
    time = DriverStation.getMatchTime();
    Optional<Alliance> ally = DriverStation.getAlliance();
    Alliance alliance = GameState.getAlliance(DriverStation.getAlliance());
    boolean wonAuto = GameState.getWonAuto(DriverStation.getGameSpecificMessage(), alliance);
    if (ally.isPresent() && DriverStation.isTeleop()) {

      if (((time < 33) && (time > 30))
          || ((time < 57) && (time > 55))
          || ((time < 83) && (time > 80))
          || ((time < 107) && (time > 105))
          || ((time < 133) && (time > 130))) {
        led.setPattern(hubSwitchingPattern);
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
