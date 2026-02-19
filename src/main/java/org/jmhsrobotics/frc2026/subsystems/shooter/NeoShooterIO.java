package org.jmhsrobotics.frc2026.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoShooterIO implements ShooterIO {
  private SparkMax leftFlywheelMotorLeader =
      new SparkMax(Constants.CAN.kLeftFlywheelMotorID, MotorType.kBrushless);
  private SparkMax centerFlywheelMotor =
      new SparkMax(Constants.CAN.kCenterFlywheelMotorID, MotorType.kBrushless);
  private SparkMax rightFlywheelMotor =
      new SparkMax(Constants.CAN.kRightFlywheelMotorID, MotorType.kBrushless);

  private SparkMaxConfig motorConfigLeader;
  private SparkMaxConfig motorConfigFollower;
  private RelativeEncoder leftFlywheelEncoder = leftFlywheelMotorLeader.getEncoder();
  private RelativeEncoder centerFlywheelEncoder = centerFlywheelMotor.getEncoder();
  private RelativeEncoder rightFlywheelEncoder = rightFlywheelMotor.getEncoder();

  private SparkClosedLoopController leftFlywheelPIDController =
      leftFlywheelMotorLeader.getClosedLoopController();
  private SparkClosedLoopController centerFlywheelPIDController =
      centerFlywheelMotor.getClosedLoopController();
  private SparkClosedLoopController rightFlywheelPIDController =
      rightFlywheelMotor.getClosedLoopController();
  private double velocityRPM;
  private double goalRPM;

  public NeoShooterIO() {

    // leftFlywheelMotor
    motorConfigLeader = new SparkMaxConfig();
    motorConfigLeader
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .voltageCompensation(12)
        .inverted(false)
        .closedLoop
        .pid(
            Constants.ShooterConstants.kP,
            Constants.ShooterConstants.kI,
            Constants.ShooterConstants.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxOutput(1)
        .minOutput(0);
    SparkUtil.tryUntilOk(
        leftFlywheelMotorLeader,
        5,
        () ->
            leftFlywheelMotorLeader.configure(
                motorConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    motorConfigFollower = new SparkMaxConfig();
    motorConfigFollower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(50)
        .voltageCompensation(12)
        .follow(leftFlywheelMotorLeader)
        .inverted(false);
    SparkUtil.tryUntilOk(
        centerFlywheelMotor,
        5,
        () ->
            centerFlywheelMotor.configure(
                motorConfigFollower,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        rightFlywheelMotor,
        5,
        () ->
            rightFlywheelMotor.configure(
                motorConfigFollower,
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
    SparkUtil.ifOk(
        leftFlywheelMotorLeader,
        leftFlywheelMotorLeader::getOutputCurrent,
        (value) -> inputs.currentAMPS = value);
    SparkUtil.ifOk(
        leftFlywheelMotorLeader,
        leftFlywheelEncoder::getVelocity,
        (value) -> inputs.velocityRPM = value);
    SparkUtil.ifOk(
        leftFlywheelMotorLeader,
        leftFlywheelMotorLeader::getBusVoltage,
        (value) -> inputs.voltage = value);
    SparkUtil.ifOk(
        leftFlywheelMotorLeader,
        leftFlywheelMotorLeader::getMotorTemperature,
        (value) -> inputs.tempC = value);

    inputs.goalRPM = this.goalRPM;
  }

  @Override
  public void setRPM(double velocityRPM) {
    this.goalRPM = velocityRPM;
    leftFlywheelPIDController.setSetpoint(velocityRPM, ControlType.kVelocity);
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
    leftFlywheelPIDController.setSetpoint(0, ControlType.kVelocity);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        leftFlywheelMotorLeader,
        5,
        () ->
            leftFlywheelMotorLeader.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
