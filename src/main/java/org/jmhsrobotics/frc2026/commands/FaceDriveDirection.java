package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private final PIDController thetaController = new PIDController(3, 0, 0);

  private boolean isFlipped;
  private Rotation2d targetHeading = new Rotation2d(); // Store the last valid heading

  public FaceDriveDirection(Drive drive, ControlBoard control) {
    this.drive = drive;
    this.control = control;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    thetaController.reset();
    // Use Radians for continuous input: -Pi to Pi
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Set initial target to current heading so it doesn't snap when command starts
    targetHeading = drive.getPose().getRotation();
  }

  @Override
  public void execute() {
    // FIXME: When fixed drivetrain branch gets merged this will need to be refactored to use new
    // constants
    double xSpeed =
        MathUtil.applyDeadband(
            this.getSquareInput(-this.control.translationY())
                * DriveConstants.intakeMaxSpeedMetersPerSec,
            DriveConstants.deadBand);

    double ySpeed =
        MathUtil.applyDeadband(
            this.getSquareInput(-this.control.translationX())
                * DriveConstants.intakeMaxSpeedMetersPerSec,
            DriveConstants.deadBand);

    isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // 1. Determine direction based on joystick vectors
    // We only update the target heading if we are actually pushing the sticks
    if (Math.hypot(xSpeed, ySpeed) > 0.1) {
      targetHeading = new Rotation2d(xSpeed, ySpeed);
    }

    Logger.recordOutput("Drive/IntakeDrive/TargetAngleDegrees", targetHeading.getDegrees());

    // 2. Calculate PID output using radians
    double thetaOutput =
        thetaController.calculate(
            (isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation())
                .getRadians(), // Measurement
            targetHeading.getRadians() // Setpoint
            );

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaOutput);

    Logger.recordOutput("Drive/IntakeDrive/isRed", isFlipped);

    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    this.drive.runVelocity(speeds);
  }

  private double getSquareInput(double input) {
    return Math.pow(input, 2) * Math.signum(input);
  }
}
