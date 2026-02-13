package org.jmhsrobotics.frc2026.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoIntakeIO implements IntakeIO {
  private SparkMax intakeMotor = new SparkMax(Constants.CAN.kIntakeMotorID, MotorType.kBrushless);
  private SparkMaxConfig intakeMotorConfig;
  private SparkMax slapDownMotor =
      new SparkMax(Constants.CAN.kSlapDownMotorID, MotorType.kBrushless);
  private SparkMaxConfig slapDownMotorConfig;
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private RelativeEncoder slapDownEncoder = slapDownMotor.getEncoder();
  private AbsoluteEncoderConfig slapDownEncoderConfig = new AbsoluteEncoderConfig();
  private SparkClosedLoopController slapDownPIDController;
  private double speedRPM;
  private double speedDutyCycle;

  private double setPointDegrees = Constants.Intake.kSlapDownUpPositionDegrees;

  public NeoIntakeIO() {

    slapDownEncoderConfig
        .positionConversionFactor(360)
        .velocityConversionFactor(6); // TODO update these values
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

    // SlapDown motor
    slapDownMotorConfig = new SparkMaxConfig();
    slapDownMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .voltageCompensation(12)
        .inverted(false)
        .signals
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    slapDownMotorConfig
        .closedLoop
        .pid(Constants.Intake.kSlapdownP, Constants.Intake.kSlapdownI, Constants.Intake.kSlapdownD)
        .outputRange(-180, 180) // TODO update to real values
        .maxMotion
        .cruiseVelocity(2)
        .maxAcceleration(12);
    slapDownMotorConfig.absoluteEncoder.apply(slapDownEncoderConfig);

    SparkUtil.tryUntilOk(
        slapDownMotor,
        5,
        () ->
            slapDownMotor.configure(
                slapDownMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    slapDownPIDController = slapDownMotor.getClosedLoopController();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;

    // slapdown
    SparkUtil.ifOk(
        slapDownMotor,
        slapDownEncoder::getPosition,
        (value) -> inputs.slapDownPositionDegrees = value);
    SparkUtil.ifOk(
        slapDownMotor,
        slapDownMotor::getOutputCurrent,
        (value) -> inputs.slapDownCurrentAmps = value);

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

  public void setPositionDegrees(double degrees) {
    slapDownPIDController.setSetpoint(degrees, ControlType.kPosition);
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

  @Override
  public void setSlapDownBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        slapDownMotor,
        5,
        () ->
            slapDownMotor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
