package org.jmhsrobotics.frc2026.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoIndexerDoryIO implements IndexerIO {
  private SparkMax motor = new SparkMax(Constants.DoryCAN.kIndexerMotorID, MotorType.kBrushless);
  private SparkMaxConfig motorConfig;
  private RelativeEncoder encoder = motor.getEncoder();
  private double speedRPM;
  private double speedDutyCycle;

  public NeoIndexerDoryIO() {
    motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAMPS = value);
    SparkUtil.ifOk(
        motor, motor::getMotorTemperature, (value) -> inputs.motorTemperatureCelcius = value);
    SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.motorRPM = value);
    inputs.outputSpeedDutyCycle = this.speedDutyCycle;
  }

  public void set(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    motor.set(speedDutyCycle);
  }

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
