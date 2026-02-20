package org.jmhsrobotics.frc2026.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoIntakeIO implements IntakeIO {
  private SparkMax intakeMotor = new SparkMax(Constants.CAN.kIntakeMotorID, MotorType.kBrushless);
  private SparkMaxConfig intakeMotorConfig;
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private double speedDutyCycle;

  public NeoIntakeIO() {
    // intakeMotor

    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30)
        .voltageCompensation(12)
        .inverted(false);

    SparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;

    // intake
    SparkUtil.ifOk(
        intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);
    SparkUtil.ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.RPM = value);
    SparkUtil.ifOk(
        intakeMotor,
        intakeMotor::getMotorTemperature,
        (value) -> inputs.intakeMotorTemperatureCelcius = value);
  }

  @Override
  public void setSpeedDutyCycle(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    intakeMotor.set(speedDutyCycle);
  }

  @Override
  public void setIntakeBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
