package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class SystemTestCommand extends Command {
  private Shooter shooter;

  public SystemTestCommand(Shooter shooter) {

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
