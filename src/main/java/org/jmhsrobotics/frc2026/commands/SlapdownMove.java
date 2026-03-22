package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;

public class SlapdownMove extends Command {
  private Slapdown slapdown;
  private double goalDegrees;

  public SlapdownMove(Slapdown slapdown, double goalDegrees) {
    this.slapdown = slapdown;
    this.goalDegrees = goalDegrees;

    addRequirements(this.slapdown);
  }

  @Override
  public void initialize() {
    this.slapdown.setPositionDegrees(goalDegrees);
  }

  @Override
  public boolean isFinished() {
    return this.slapdown.atGoal();
  }
}
