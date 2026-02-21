package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class ShooterSpinupCopy extends Command {
  private Shooter shooter;
  private double speed;

  public ShooterSpinupCopy(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;

    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    // this.shooter.setRPM(0);
  }

  @Override
  public void execute() {
    this.shooter.setSpeed(this.speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.stop(); // Do not use Power to Spindown flywheels
  }
}
