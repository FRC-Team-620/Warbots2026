package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.climber.Climber;

public class ClimberMove extends Command {
  private Climber climber;
  private double goalPositionCM;

  public ClimberMove(Climber climber, double goalPositionCM) {
    this.climber = climber;
    this.goalPositionCM = goalPositionCM;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setPositionCM(goalPositionCM);
  }
}
