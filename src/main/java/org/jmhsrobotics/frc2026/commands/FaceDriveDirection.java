package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class FaceDriveDirection extends Command {
  private final Drive drive;
  private final ControlBoard control;
  private final PIDController thetaController = new PIDController(0.1, 0, 0);
  private boolean isFlipped;

  public FaceDriveDirection(Drive drive, ControlBoard control) {
    this.drive = drive;
    this.control = control;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.enableContinuousInput(-180, 180);
  }

  public void execute() {
    double xSpeed, ySpeed;

    // FIXME: When fixed drivetrain branch gets merged this will need to be refactored to use new
    // contants
    xSpeed =
        MathUtil.applyDeadband(
            this.getSquareInput(-this.control.translationY()) * DriveConstants.slowdownCoefficient,
            DriveConstants.deadBand);

    ySpeed =
        MathUtil.applyDeadband(
            this.getSquareInput(-this.control.translationX()) * DriveConstants.slowdownCoefficient,
            DriveConstants.deadBand);
    isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d goalPose = calculateSetpoint();
    Rotation2d goalRotation = goalPose.getRotation();

    Logger.recordOutput("Drive/IntakeDrive/GoalPose", goalPose);

    thetaController.setSetpoint(goalPose.getRotation().getDegrees());

    Logger.recordOutput("Drive/IntakeDrive/GoalAngleDegrees", thetaController.getSetpoint());


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
  }

  public Pose2d calculateSetpoint() {
    Translation2d robotTranslation2d = drive.getPose().getTranslation();
    Rotation2d robotRotation2d = drive.getPose().getRotation();

    Rotation2d targetAngle = drive.getPose().getTranslation().getAngle();

    Logger.recordOutput("Drive/IntakeDrive/TargetAngle", targetAngle);

    Rotation2d rotationRequired = targetAngle.minus(robotRotation2d);
    return new Pose2d(drive.getPose().getX(), drive.getPose().getY(), rotationRequired);
  }

  private double getSquareInput(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }
}
