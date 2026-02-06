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
  private SparkMax motor = new SparkMax(Constants.CAN.kFlywheelMotorID, MotorType.kBrushless);
  private RelativeEncoder encoder;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController pidController;
  private double velocityRPM;

  public NeoShooterIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(25)
        .voltageCompensation(12)
        .inverted(true)
        .closedLoop
        .pid(
            Constants.ShooterConstants.kP,
            Constants.ShooterConstants.kI,
            Constants.ShooterConstants.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .maxOutput(1)
        .minOutput(0)
        .maxMotion
        .cruiseVelocity(9800)
        .maxAcceleration(20000);
    // TODO set motorConfig values
    // Initialize the closed loop controller
    pidController = motor.getClosedLoopController();

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAMPS = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    SparkUtil.ifOk(motor, motor::getBusVoltage, (value) -> inputs.voltage = value);
    SparkUtil.ifOk(motor, motor::getMotorTemperature, (value) -> inputs.tempC = value);
    pidController.setSetpoint(this.velocityRPM, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void set(double velocityRPM) {
    this.velocityRPM = velocityRPM;
  }

  @Override
  public void setPIDF(double p, double i, double d, double f) {
    // TODO: Implement ff term
    // FIXME:
    var setPIDF = new SparkMaxConfig();
    setPIDF.closedLoop.pid(p, i, d);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                setPIDF, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
