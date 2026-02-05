package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
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
  public void execute() {
    double rightTrigger = rightTriggerAxis.getAsDouble();
    this.shooter.setVelocity(rightTrigger * 0.3);

    if (rightTrigger == 0) {
      this.shooter.setVelocity(0.1);
    }
  }

  @Override
  public void initialize() {
    this.shooter.setVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.shooter.setVelocity(0);
  }
}
