package org.jmhsrobotics.frc2026.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoClimberIO implements ClimberIO {

  private SparkMax climberMotor =
      new SparkMax(Constants.CAN.kClimberMotorID, SparkMax.MotorType.kBrushless);
  private SparkMaxConfig climberMotorConfig;
  private double speedDutyCycle;

  public NeoClimberIO() {
    climberMotorConfig = new SparkMaxConfig();
    climberMotorConfig
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        climberMotor,
        5,
        () ->
            climberMotor.configure(
                climberMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    SparkUtil.ifOk(
        climberMotor,
        climberMotor.getEncoder()::getPosition,
        (value) -> inputs.motorPositionCM = value * Constants.Climber.kCMPerRotation);
    SparkUtil.ifOk(
        climberMotor, climberMotor::getOutputCurrent, (value) -> inputs.currentAMPS = value);
    SparkUtil.ifOk(
        climberMotor,
        climberMotor::getMotorTemperature,
        (value) -> inputs.motorTemperatureCelcius = value);

    inputs.motorSpeedDutyCycle = this.speedDutyCycle;
  }

  @Override
  public void setSpeedDutyCycle(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    climberMotor.set(speedDutyCycle);
  }
}
