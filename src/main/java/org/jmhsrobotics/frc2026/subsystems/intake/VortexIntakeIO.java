package org.jmhsrobotics.frc2026.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class VortexIntakeIO implements IntakeIO {
  private SparkFlex intakeLeaderMotor =
      new SparkFlex(Constants.CAN.kIntakeMotorID, MotorType.kBrushless);
  private SparkFlex intakeFollowerMotor =
      new SparkFlex(Constants.CAN.kIntakeFollowerMotorID, MotorType.kBrushless);
  private SparkFlexConfig intakeLeaderMotorConfig;
  private SparkFlexConfig intakeFollowerMotorConfig;
  private RelativeEncoder intakeEncoder = intakeLeaderMotor.getEncoder();
  private double speedDutyCycle;

  public VortexIntakeIO() {
    // Create the leader configuration
    intakeLeaderMotorConfig = new SparkFlexConfig();
    intakeLeaderMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .voltageCompensation(12)
        .inverted(false);

    // Create the follower configuration
    intakeFollowerMotorConfig = new SparkFlexConfig();
    intakeFollowerMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(30)
        .voltageCompensation(12)
        .follow(Constants.CAN.kIntakeMotorID, true);

    // Create the leader instance
    SparkUtil.tryUntilOk(
        intakeLeaderMotor,
        5,
        () ->
            intakeLeaderMotor.configure(
                intakeLeaderMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    // Create the follower instance
    SparkUtil.tryUntilOk(
        intakeFollowerMotor,
        5,
        () ->
            intakeFollowerMotor.configure(
                intakeFollowerMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;

    // Get the current values from the leader motor
    SparkUtil.ifOk(
        intakeLeaderMotor,
        intakeLeaderMotor::getOutputCurrent,
        (value) -> inputs.intakeCurrentAmps = value);
    SparkUtil.ifOk(intakeLeaderMotor, intakeEncoder::getVelocity, (value) -> inputs.RPM = value);
    SparkUtil.ifOk(
        intakeLeaderMotor,
        intakeLeaderMotor::getMotorTemperature,
        (value) -> inputs.intakeMotorTemperatureCelcius = value);

    // Get the current values from the follower motor
    SparkUtil.ifOk(
        intakeFollowerMotor,
        intakeFollowerMotor::getOutputCurrent,
        (value) -> inputs.intakeCurrentAmps = value);
    SparkUtil.ifOk(intakeFollowerMotor, intakeEncoder::getVelocity, (value) -> inputs.RPM = value);
    SparkUtil.ifOk(
        intakeFollowerMotor,
        intakeFollowerMotor::getMotorTemperature,
        (value) -> inputs.intakeMotorTemperatureCelcius = value);

    // See if the motor is currently stalled
    boolean isStalled =
        Math.abs(speedDutyCycle) > 0.1
            && Math.abs(intakeLeaderMotor.getEncoder().getVelocity()) < 10.0;
    inputs.stalled = isStalled;
  }

  @Override
  public void setSpeedDutyCycle(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    intakeLeaderMotor.set(speedDutyCycle);
  }

  @Override
  public void setIntakeBrakeMode(boolean enable) {
    // Set the brake config
    var brakeConfig = new SparkFlexConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);

    // Set the leader motor
    SparkUtil.tryUntilOk(
        intakeLeaderMotor,
        5,
        () ->
            intakeLeaderMotor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

    // Set the follower motor
    SparkUtil.tryUntilOk(
        intakeFollowerMotor,
        5,
        () ->
            intakeFollowerMotor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
