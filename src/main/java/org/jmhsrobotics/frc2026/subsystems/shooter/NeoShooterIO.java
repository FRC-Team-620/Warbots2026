package org.jmhsrobotics.frc2026.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
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

  public NeoShooterIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(25)
        .voltageCompensation(12)
        .inverted(true)
        .closedLoop
        .pid(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
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
  }

  @Override
  public void setRPM(double velocityRPM) {
    pidController.setSetpoint(velocityRPM, ControlType.kVelocity);
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
