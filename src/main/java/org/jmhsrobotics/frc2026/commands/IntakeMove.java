package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;

public class IntakeMove extends Command {
  private Intake intake;

  public IntakeMove(Intake intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.set(0);
  }

  @Override
  public void execute() {
    intake.set(Constants.Intake.kBaseRPM);
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
