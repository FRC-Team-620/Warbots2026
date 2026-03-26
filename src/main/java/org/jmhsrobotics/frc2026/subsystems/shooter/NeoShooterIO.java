package org.jmhsrobotics.frc2026.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;
import org.littletonrobotics.junction.Logger;

public class NeoShooterIO implements ShooterIO {
  private SparkMax leftFlywheelMotor =
      new SparkMax(Constants.CAN.kLeftFlywheelMotorID, MotorType.kBrushless);
  private SparkFlex centerFlywheelMotor =
      new SparkFlex(Constants.CAN.kCenterFlywheelMotorID, MotorType.kBrushless);
  private SparkMax rightFlywheelMotor =
      new SparkMax(Constants.CAN.kRightFlywheelMotorID, MotorType.kBrushless);

  private SparkMaxConfig motorConfigLeftFollower;
  private SparkMaxConfig motorConfigRightFollower;
  private SparkFlexConfig motorConfigMiddleLeader;
  private RelativeEncoder leftFlywheelEncoder = leftFlywheelMotor.getEncoder();
  private RelativeEncoder centerFlywheelEncoder = centerFlywheelMotor.getEncoder();
  private RelativeEncoder rightFlywheelEncoder = rightFlywheelMotor.getEncoder();
  private double voltage;

  private SparkClosedLoopController leftFlywheelPIDController =
      leftFlywheelMotor.getClosedLoopController();
  private SparkClosedLoopController centerFlywheelPIDController =
      centerFlywheelMotor.getClosedLoopController();
  private SparkClosedLoopController rightFlywheelPIDController =
      rightFlywheelMotor.getClosedLoopController();

  private double velocityRPM;
  private double goalRPM;

  public NeoShooterIO() {
    // leftFlywheelMotor
    motorConfigLeftFollower = new SparkMaxConfig();
    motorConfigLeftFollower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .follow(Constants.CAN.kCenterFlywheelMotorID, false)
        //  .voltageCompensation(12)
        .inverted(true)
        .closedLoop
        .pid(
            Constants.ShooterConstants.kP,
            Constants.ShooterConstants.kI,
            Constants.ShooterConstants.kD)
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxOutput(1)
        .minOutput(0);

    motorConfigRightFollower = new SparkMaxConfig();
    motorConfigRightFollower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .inverted(false)
        .follow(Constants.CAN.kCenterFlywheelMotorID, true)
        .closedLoop
        .pid(
            Constants.ShooterConstants.kP,
            Constants.ShooterConstants.kI,
            Constants.ShooterConstants.kD);
    // .voltageCompensation(12);
    //    .follow(leftFlywheelMotorLeader, true);

    motorConfigMiddleLeader = new SparkFlexConfig();
    motorConfigMiddleLeader
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .inverted(true)
        .closedLoop
        .pid(
            Constants.ShooterConstants.kP,
            Constants.ShooterConstants.kI,
            Constants.ShooterConstants.kD);
    // .voltageCompensation(12);
    // .follow(leftFlywheelMotorLeader, false);
    // motorConfigMiddleLeader
    //     .encoder
    //     .quadratureMeasurementPeriod(5)
    //     .quadratureAverageDepth(32)
    //     .uvwMeasurementPeriod(8)
    //     .uvwAverageDepth(2);

    SparkUtil.tryUntilOk(
        leftFlywheelMotor,
        5,
        () ->
            leftFlywheelMotor.configure(
                motorConfigLeftFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        centerFlywheelMotor,
        5,
        () ->
            centerFlywheelMotor.configure(
                motorConfigMiddleLeader,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    SparkUtil.tryUntilOk(
        rightFlywheelMotor,
        5,
        () ->
            rightFlywheelMotor.configure(
                motorConfigRightFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    // TODO set motorConfig values
    // Initialize the closed loop controller
    // motorConfig.follow(leftFlywheelMotorLeader)
    // pidController = motor.getClosedLoopController();

    // SparkUtil.tryUntilOk(
    //     motor,
    //     5,
    //     () ->
    //         motor.configure(
    //             motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // leftFlywheelEncoder = leftFlywheelMotorLeader.getAbsoluteEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // FIXME: Should make an array for each individaul input that contains values from all 3 motors,
    // then return arrays
    // SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAMPS
    // = value);
    // SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRPM =
    // value);
    // SparkUtil.ifOk(motor, motor::getBusVoltage, (value) -> inputs.voltage =
    // value);
    // SparkUtil.ifOk(motor, motor::getMotorTemperature, (value) -> inputs.tempC =
    // value);
    // pidController.setSetpoint(this.velocityRPM,
    // ControlType.kMAXMotionVelocityControl);

    // SparkUtil.ifOk(
    //     leftFlywheelMotor,
    //     leftFlywheelMotor::getOutputCurrent,
    //     (value) -> inputs.currentAMPS = value);
    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelEncoder::getVelocity, (value) -> inputs.velocityRPM =
    // value);

    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelEncoder::getPosition, (value) -> inputs.positionROT =
    // value);
    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelMotor::getBusVoltage, (value) -> inputs.voltage = value);
    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelMotor::getMotorTemperature, (value) -> inputs.tempC =
    // value);

    SparkUtil.ifOk(
        centerFlywheelMotor,
        centerFlywheelMotor::getOutputCurrent,
        (value) -> inputs.currentAMPS = value);
    SparkUtil.ifOk(
        centerFlywheelMotor,
        centerFlywheelEncoder::getVelocity,
        (value) -> inputs.velocityRPM = value);

    SparkUtil.ifOk(
        centerFlywheelMotor,
        centerFlywheelEncoder::getPosition,
        (value) -> inputs.positionROT = value);
    SparkUtil.ifOk(
        centerFlywheelMotor, centerFlywheelMotor::getBusVoltage, (value) -> inputs.voltage = value);
    SparkUtil.ifOk(
        centerFlywheelMotor,
        centerFlywheelMotor::getMotorTemperature,
        (value) -> inputs.tempC = value);

    Logger.recordOutput("LeftFlywheelVelocity", leftFlywheelEncoder.getVelocity());
    Logger.recordOutput("CenterFlywheelVelocity", centerFlywheelEncoder.getVelocity());
    Logger.recordOutput("RightFlywheelVelocity", rightFlywheelEncoder.getVelocity());

    inputs.appliedVoltage = voltage;
    inputs.goalRPM = this.goalRPM;
  }

  @Override
  public void setRPM(double velocityRPM) {
    this.goalRPM = velocityRPM;
    // leftFlywheelPIDController.setSetpoint(velocityRPM, ControlType.kVelocity);
    centerFlywheelPIDController.setSetpoint(velocityRPM, ControlType.kVelocity);
    //  rightFlywheelPIDController.setSetpoint(velocityRPM, ControlType.kVelocity);
  }

  @Override
  public void setSpeed(double speed) {
    centerFlywheelMotor.set(speed);
  }

  @Override
  public void setVoltage(double voltage) {
    centerFlywheelMotor.setVoltage(voltage);
    this.voltage = voltage;
  }

  @Override
  public void setPIDF(double p, double i, double d, double f) {
    // TODO: Implement ff term
    // FIXME:
    // var setPIDF = new SparkMaxConfig();
    // setPIDF.closedLoop.pid(p, i, d);
    // SparkUtil.tryUntilOk(
    // motor,
    // 5,
    // () ->
    // motor.configure(
    // setPIDF, ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters));
  }

  @Override
  public void stop() {
    // leftFlywheelPIDController.setSetpoint(0, ControlType.kVelocity);
    centerFlywheelMotor.stopMotor();
  }

  // TODO: Change left flywheel leader in brake mode to center
  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        leftFlywheelMotor,
        5,
        () ->
            leftFlywheelMotor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
