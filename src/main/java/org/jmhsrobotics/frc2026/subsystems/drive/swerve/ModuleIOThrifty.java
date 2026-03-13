// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2026.subsystems.drive.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Queue;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2026.subsystems.drive.DriveConstants;
import org.jmhsrobotics.frc2026.subsystems.drive.DriveConstants.thriftyConstants;
import org.jmhsrobotics.frc2026.subsystems.drive.SparkOdometryThread;
import org.jmhsrobotics.frc2026.util.SparkUtil;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOThrifty implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase driveSpark;
  private final SparkBase turnSpark;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  private boolean stopped = false;

  public ModuleIOThrifty(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> DriveConstants.frontLeftZeroRotation;
          case 1 -> DriveConstants.frontRightZeroRotation;
          case 2 -> DriveConstants.backLeftZeroRotation;
          case 3 -> DriveConstants.backRightZeroRotation;
          default -> new Rotation2d();
        };
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> thriftyConstants.frontLeftDriveCanId;
              case 1 -> thriftyConstants.frontRightDriveCanId;
              case 2 -> thriftyConstants.backLeftDriveCanId;
              case 3 -> thriftyConstants.backRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> thriftyConstants.frontLeftTurnCanId;
              case 1 -> thriftyConstants.frontRightTurnCanId;
              case 2 -> thriftyConstants.backLeftTurnCanId;
              case 3 -> thriftyConstants.backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // FIXME: remove and set these after tuning complete
    SmartDashboard.putNumber("DriveTuning/defaultMaxSpeedMPS", 3);
    SmartDashboard.putNumber("DriveTuning/turboMaxSpeedMPS", 4);
    SmartDashboard.putNumber("DriveTuning/intakeMaxSpeedMPS", 2);
    SmartDashboard.putNumber("DriveTuning/autoMaxSpeedMPS", 3);
    SmartDashboard.putNumber("DriveTuning/defaultMaxRotMPS", 3);
    SmartDashboard.putNumber("DriveTuning/turboMaxRotMPS", 3);
    SmartDashboard.putNumber("DriveTuning/intakeMaxRotMPS", 3);
    SmartDashboard.putNumber("DriveTuning/autoMaxRotMPS", 3);

    SmartDashboard.putBoolean("DriveTuning/slewRateEnabled", true);
    SmartDashboard.putNumber("DriveTuning/slewRatePeriodSecs", 0.3);
    SmartDashboard.putNumber("DriveTuning/rotSlewRatePeriodSecs", 0.2);

    SmartDashboard.putNumber("DriveTuning/Speed-Kp", 0);
    SmartDashboard.putNumber("DriveTuning/Speed-Ki", 0);
    SmartDashboard.putNumber("DriveTuning/Speed-Kd", 0);

    SmartDashboard.putNumber("DriveTuning/Rot-Kp", 0);
    SmartDashboard.putNumber("DriveTuning/Rot-Ki", 0);
    SmartDashboard.putNumber("DriveTuning/Rot-Kd", 0);

    // Configure drive motor
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(thriftyConstants.driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(thriftyConstants.driveEncoderPositionFactor)
        .velocityConversionFactor(thriftyConstants.driveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig.closedLoop.pid(
        // FIXME: switch back to constants after tuning is finished
        SmartDashboard.getNumber("DriveTuning/Speed-Kp", 0),
        SmartDashboard.getNumber("DriveTuning/Speed-Ki", 0),
        SmartDashboard.getNumber("DriveTuning/Speed-Kd", 0));
    // thriftyConstants.driveKp, thriftyConstants.driveKi, thriftyConstants.driveKd);

    // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(thriftyConstants.turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(thriftyConstants.turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .absoluteEncoder
        .inverted(thriftyConstants.turnEncoderInverted)
        .positionConversionFactor(thriftyConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(thriftyConstants.turnEncoderVelocityFactor)
        .averageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(
            thriftyConstants.turnPIDMinInput, thriftyConstants.turnPIDMaxInput)
        // .pid(thriftyConstants.turnKp, thriftyConstants.turnKi, thriftyConstants.turnKd);
        // FIXME: switch back to constants after tuning is finished
        .pid(
            SmartDashboard.getNumber("DriveTuning/Rot-Kp", 0),
            SmartDashboard.getNumber("DriveTuning/Rot-Ki", 0),
            SmartDashboard.getNumber("DriveTuning/Rot-Kd", 0));
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    SparkUtil.tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(
        driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    SparkUtil.ifOk(
        driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    SparkUtil.ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    // Update turn inputs
    SparkUtil.sparkStickyFault = false;
    SparkUtil.ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    SparkUtil.ifOk(
        turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    SparkUtil.ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    SparkUtil.ifOk(
        turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts =
        thriftyConstants.driveKs * Math.signum(velocityRadPerSec)
            + thriftyConstants.driveKv * velocityRadPerSec;
    driveController.setSetpoint(
        velocityRadPerSec,
        ControlType.kVelocity,
        this.stopped ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(),
            thriftyConstants.turnPIDMinInput,
            thriftyConstants.turnPIDMaxInput);
    turnController.setSetpoint(setpoint, ControlType.kPosition);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    SparkUtil.tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void stoppedDrivePID() {
    stopped = true;
  }

  @Override
  public void movingDrivePID() {
    stopped = false;
  }

  @Override
  public boolean stoppedP() {
    return stopped;
  }
}
