package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class ShooterMove extends Command {
  private Shooter shooter;
  private DoubleSupplier rightTriggerAxis;

  public ShooterMove(Shooter shooter, DoubleSupplier rightTriggerAxis) {
    this.shooter = shooter;
    this.rightTriggerAxis = rightTriggerAxis;

    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.shooter.setRPM(0);
  }

  @Override
  public void execute() {
    double rightTrigger = rightTriggerAxis.getAsDouble();
    if (rightTrigger > 0) {
      this.shooter.setRPM(Constants.ShooterConstants.kBaseRPM);
    } else {
      // this.shooter.setRPM(0.0);
      this.shooter.stop(); //Do not use Power to Spindown flywheels
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.stop(); //Do not use Power to Spindown flywheels
    // this.shooter.setRPM(0);
  }
}
