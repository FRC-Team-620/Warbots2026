package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.FieldConstants;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class DistanceAdjustingShoot extends Command {
  private Shooter shooter;
  private double goalRPM;
  private Drive drive;
  private double distance;

  public DistanceAdjustingShoot(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;

    addRequirements(this.shooter);
  }

  @Override
  public void initialize() {
    this.shooter.setRPM(0);
  }

  @Override
  public void execute() {

    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Blue) {
      this.distance =
          this.getDistance(
              FieldConstants.Hub.topCenterPoint.toTranslation2d(),
              drive.getPose().getTranslation());
    } else {
      this.distance =
          this.getDistance(
              FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(),
              drive.getPose().getTranslation());
    }
    double goalRPM = this.shooter.calculateEstimatedRPM(distance);
    Logger.recordOutput("Shooter/DistanceAdjustedGoal", goalRPM);
    double hoodPosition = this.shooter.calculateHoodPosition(distance);
    Logger.recordOutput("Shooter/Hood Position", hoodPosition);
    this.shooter.setRPM(goalRPM);
    this.shooter.setHoodPosition(hoodPosition);
    // if (goalRPM > 0) {
    //   this.shooter.setRPM(goalRPM);
    // } else {
    //   this.shooter.stop(); // Do not use Power to Spindown flywheels
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

  public double getDistance(Translation2d goalTrans, Translation2d robotTranslation) {
    double currentDistance;
    currentDistance =
        Math.sqrt(
            Math.pow(goalTrans.getX() - robotTranslation.getX(), 2)
                + Math.pow(goalTrans.getY() - robotTranslation.getY(), 2));

    return currentDistance;
  }
}
