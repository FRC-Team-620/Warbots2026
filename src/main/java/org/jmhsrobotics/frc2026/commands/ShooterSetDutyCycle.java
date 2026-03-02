package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class ShooterSetDutyCycle extends Command {
  private Shooter shooter;

  private double speed;
  public static double jankSpeed = 0.6;

  public ShooterSetDutyCycle(Shooter shooter, double speed) {
    this.shooter = shooter;
    this.speed = speed;
    SmartDashboard.putNumber("jankSpeed", jankSpeed);

    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    // this.shooter.setRPM(0);
  }

  @Override
  public void execute() {
    // jankSpeed = SmartDashboard.getNumber("jankSpeed", 0.6);
    // if (this.speed != 0) {
    //   this.shooter.setSpeed(jankSpeed);
    // } else {
    this.shooter.setSpeed(this.speed);
    // }
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
