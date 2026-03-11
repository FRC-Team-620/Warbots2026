package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.FieldConstants;
import org.jmhsrobotics.frc2026.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class AlignToHubInAuto extends Command {

  private final Drive drive;
  private final ControlBoard control;
  private final PIDController thetaController = new PIDController(0.1, 0, 0);

  private boolean isRedAlliance;
  private boolean isTeleop;

  private double thetaGoalDegrees = 0;

  private int targetTagId;
  private Pose3d lastTagPose = null;
  private Pose3d tagPose;
  private boolean isFlipped;

  public AlignToHubInAuto(Drive drive, ControlBoard control) {
    this.drive = drive;
    this.control = control;
    this.isTeleop = false;

    addRequirements(drive);
  }

  @Override
  public void initialize() {

    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);
  }

  public void execute() {
    double xSpeed, ySpeed;

    // Add TurboMode

    xSpeed =
        MathUtil.applyDeadband(
            this.getSquareInput(-this.control.translationY()) * DriveConstants.autoMaxSpeedMetersPerSec,
            DriveConstants.deadBand);
    ySpeed =
        MathUtil.applyDeadband(
            this.getSquareInput(-this.control.translationX()) * DriveConstants.autoMaxSpeedMetersPerSec,
            DriveConstants.deadBand);

    isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d goalPose = calculateSetpoints();
    Rotation2d goalRotation = goalPose.getRotation();

    Logger.recordOutput("Align/GoalPose", goalPose);

    thetaController.setSetpoint(goalPose.getRotation().getDegrees());

    double thetaOutput =
        thetaController.calculate(
            drive.getPose().getRotation().getDegrees(),
            drive.getPose().getRotation().getDegrees() + goalRotation.getDegrees());

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaOutput);

    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    this.drive.runVelocity(speeds);

    Logger.recordOutput("Align/GoalAngleDegrees", thetaController.getSetpoint());
    Logger.recordOutput(
        "Align/Distance",
        getAutoAlignDistance(
            FieldConstants.Hub.topCenterPoint.toTranslation2d(), drive.getPose().getTranslation()));
  }

  public Pose2d calculateSetpoints() {

    Translation2d tagTranslation = new Translation2d();

    Logger.recordOutput("Align/hubPoint", FieldConstants.Hub.topCenterPoint);
    if (isFlipped) {
      tagTranslation = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    } else {
      tagTranslation = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    }
    ;

    // Pose2d tagPose = new Pose2d(tagTranslation, new Rotation2d());

    Logger.recordOutput("Align/HubTagPose", tagPose);

    Translation2d robotTranslation2d = drive.getPose().getTranslation();
    Rotation2d robotRotation2d = drive.getPose().getRotation();

    // Takes the xy position of the tag and subtracts the xy position of the robot to find the
    Rotation2d targetAngle = tagTranslation.minus(robotTranslation2d).getAngle();

    Logger.recordOutput("Align/TargetAngle", targetAngle);

    Rotation2d rotationRequired = targetAngle.minus(robotRotation2d);

    // Logger.recordOutput("Feeder/Hub", tagPose);

    return new Pose2d(drive.getPose().getX(), drive.getPose().getY(), rotationRequired);
    /*return (isRedAlliance)
      ? new Pose2d(
          drive.getPose().getX(),
          drive.getPose().getY(),
          rotationRequired.plus(new Rotation2d(180)))
      : new Pose2d(drive.getPose().getX(), drive.getPose().getY(), rotationRequired);
    */
  }

  private double getSquareInput(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }

  public double getAutoAlignDistance(Translation2d goalTrans, Translation2d robotTranslation) {
    double currentDistance;
    currentDistance =
        Math.sqrt(
            Math.pow(goalTrans.getX() - robotTranslation.getX(), 2)
                + Math.pow(goalTrans.getY() - robotTranslation.getY(), 2));
    return currentDistance;
  }
}
