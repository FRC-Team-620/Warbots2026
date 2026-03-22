package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.climber.Climber;

public class ClimberExtendHooks extends Command {
  private Climber climber;
  private double goalPositionCM;

  public ClimberExtendHooks(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.extendHooks();
  }
}
