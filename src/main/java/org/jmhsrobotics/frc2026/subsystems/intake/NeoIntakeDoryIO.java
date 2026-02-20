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

public class NeoIntakeDoryIO implements IntakeIO {
  private SparkMax intakeMotor =
      new SparkMax(Constants.DoryCAN.kIntakeMotorID, MotorType.kBrushless);
  private SparkMaxConfig intakeMotorConfig;
  private SparkMaxConfig slapDownMotorConfig;
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private double speedRPM;
  private double previousRPM;

  public NeoIntakeDoryIO() {
    // intakeMotor

    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
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

    // FIXME: This should be done within the set rpm method
    // FIXME: This should be done within the set Degrees method
  }

  @Override
  public void setSpeedDutyCycle(double RPM) {
    this.speedRPM = RPM;
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
