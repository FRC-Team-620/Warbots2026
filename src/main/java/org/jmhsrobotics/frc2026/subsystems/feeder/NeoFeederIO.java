package org.jmhsrobotics.frc2026.subsystems.feeder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoFeederIO implements FeederIO {

  private double feederSpeedDutyCycle;

  private SparkMax feederMotor = new SparkMax(53, MotorType.kBrushless);
  private SparkMaxConfig feederMotorConfig;

  public NeoFeederIO() {
    feederMotorConfig = new SparkMaxConfig();
    feederMotorConfig
        .idleMode(SparkMaxConfig.IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(false);
  }

  public void updateInputs(FeederIO.FeederIOInputs inputs) {
    SparkUtil.ifOk(
        feederMotor, feederMotor::getOutputCurrent, (value) -> inputs.feederCurrentAMPS = value);
    SparkUtil.ifOk(
        feederMotor, feederMotor::getBusVoltage, (value) -> inputs.feederVoltage = value);
    SparkUtil.ifOk(
        feederMotor,
        feederMotor::getMotorTemperature,
        (value) -> inputs.feederTemperatureCelcius = value);
    inputs.feederSpeedDutyCycle = this.feederSpeedDutyCycle;
  }

  public void setFeederSpeed(double dutyCycle) {
    this.feederSpeedDutyCycle = dutyCycle;
    feederMotor.set(dutyCycle);
  }
}
