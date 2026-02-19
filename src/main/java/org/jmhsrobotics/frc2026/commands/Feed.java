package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class Feed extends Command {
  private Shooter shooter;
  private double speed;

  public Feed(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.shooter.setFeederSpeed(0);
  }

  @Override
  public void execute() {
    if (shooter.atRPMGoal()) {
      this.shooter.setFeederSpeed(this.speed);
    } else {
      this.shooter.setFeederSpeed(0);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.setFeederSpeed(0);
  }
}
