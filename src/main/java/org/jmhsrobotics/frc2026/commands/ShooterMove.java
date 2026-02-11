package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class ShooterMove extends Command {
  private Shooter shooter;
  private double goalRPM;

  public ShooterMove(Shooter shooter, double goalRPM) {
    this.shooter = shooter;
    this.goalRPM = goalRPM;

    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.shooter.setRPM(0);
  }

  @Override
  public void execute() {
    if (goalRPM > 0) {
      this.shooter.setRPM(goalRPM);
    } else {
      // this.shooter.setRPM(0.0);
      this.shooter.stop(); // Do not use Power to Spindown flywheels
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.stop(); // Do not use Power to Spindown flywheels
    // this.shooter.setRPM(0);
  }
}
