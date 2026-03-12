package org.jmhsrobotics.frc2026.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;
import org.jmhsrobotics.frc2026.subsystems.led.LED;

public class IntakeMove extends Command {
  private Intake intake;
  private double speedDutyCycle;
  private LED led;

  private LEDPattern blinkPattern = LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.1));

  public IntakeMove(Intake intake, double speedDutyCycle) {
    this.intake = intake;
    this.speedDutyCycle = speedDutyCycle;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.set(speedDutyCycle);
    led.setPattern(blinkPattern);
  }

  @Override
  public void execute() {
    intake.set(speedDutyCycle);
    led.setPattern(blinkPattern);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.println("Intake Move Complete " + interrupted);
  }
}
