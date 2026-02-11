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
  private SparkMax leftFlywheelMotorLeader = new SparkMax(Constants.CAN.kLeftFlywheelMotorID, MotorType.kBrushless);
  private SparkMax centerFlywheelMotor = new SparkMax(Constants.CAN.kCenterFlywheelMotorID, MotorType.kBrushless);
  private SparkMax rightFlywheelMotor = new SparkMax(Constants.CAN.kRightFlywheelMotorID, MotorType.kBrushless);
  private SparkMaxConfig motorConfigLeader;
  private SparkMaxConfig motorConfigFolower;
  private SparkMaxConfig rightFlywheelMotorConfig;
  private RelativeEncoder encoder;
  private SparkClosedLoopController leftFlywheelPIDController;
  private SparkClosedLoopController centerFlywheelPIDController;
  private SparkClosedLoopController rightFlywheelPIDController;
  private double velocityRPM;

  public NeoShooterIO() {


    // leftFlywheelMotor
    motorConfigLeader = new SparkMaxConfig();
    motorConfigLeader
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(25)
        .voltageCompensation(12)
        .inverted(true)
        .closedLoop
        .pid(Constants.ShooterConstants.kP, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxOutput(1)
        .minOutput(0);
      SparkUtil.tryUntilOk(leftFlywheelMotorLeader, 5, ()-> {
         leftFlywheelMotorLeader.configure(centerFlywheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
      });

      motorConfigFolower = new SparkMaxConfig();
    motorConfigFolower
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(25)
        .voltageCompensation(12)
        .follow(leftFlywheelMotorLeader)
        .inverted(true);
      SparkUtil.tryUntilOk(centerFlywheelMotor, 5, ()->
         centerFlywheelMotor.configure(motorConfigFolower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
      );
      SparkUtil.tryUntilOk(rightFlywheelMotor, 5, ()->
         rightFlywheelMotor.configure(motorConfigFolower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
      );
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

    // encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
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
  }

  @Override
  public void setRPM(double velocityRPM) {
    this.velocityRPM = velocityRPM;
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
    // motor.set(0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // var brakeConfig = new SparkMaxConfig();
    // brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    // SparkUtil.tryUntilOk(
    // motor,
    // 5,
    // () ->
    // motor.configure(
    // brakeConfig, ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters));
  }
}
