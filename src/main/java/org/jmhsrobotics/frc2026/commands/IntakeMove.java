package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;

public class IntakeMove extends Command {
  private Intake intake;
  private double speedDutyCycle;

  public IntakeMove(Intake intake, double speedDutyCycle) {
    this.intake = intake;
    this.speedDutyCycle = speedDutyCycle;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.set(speedDutyCycle);
  }

  @Override
  public void execute() {
    intake.set(speedDutyCycle);
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
