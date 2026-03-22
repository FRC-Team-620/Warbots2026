package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class HoodDown extends Command {
  private Shooter shooter;

  public HoodDown(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.shooter.setServoPosition(0.2);
  }

  @Override
  public void execute() {
    this.shooter.setServoPosition(0.2);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.setServoPosition(0.2);
  }
}
