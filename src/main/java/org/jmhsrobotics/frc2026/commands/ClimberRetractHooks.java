package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.climber.Climber;

public class ClimberRetractHooks extends Command {
  private Climber climber;
  private double goalPositionCM;

  public ClimberRetractHooks(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.retractHooks();
  }
}
