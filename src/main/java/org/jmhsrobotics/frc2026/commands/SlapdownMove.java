package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;

public class SlapdownMove extends Command {
  private Intake intake;
  private double goalDegrees;

  public SlapdownMove(Intake intake, double goalDegrees) {
    this.intake = intake;
    this.goalDegrees = goalDegrees;
    addRequirements(this.intake);
  }

  @Override
  public void initialize() {
    this.intake.setPositionDegrees(goalDegrees);
  }

  @Override
  public boolean isFinished() {
    return this.intake.atGoal();
  }
}
