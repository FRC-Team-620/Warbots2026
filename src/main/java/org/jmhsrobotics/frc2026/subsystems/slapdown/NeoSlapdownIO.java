package org.jmhsrobotics.frc2026.subsystems.slapdown;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.slapdown.SlapdownIO.SlapdownIOInputs;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class NeoSlapdownIO implements SlapdownIO {
  private SparkMax slapdownMotor =
      new SparkMax(Constants.CAN.kSlapdownMotorID, MotorType.kBrushless);
  private SparkMaxConfig slapdownMotorConfig;
  private RelativeEncoder slapdownEncoder = slapdownMotor.getEncoder();
  private AbsoluteEncoderConfig slapdownEncoderConfig = new AbsoluteEncoderConfig();
  private SparkClosedLoopController slapdownPIDController;

  private double setPointDegrees = Constants.Slapdown.kSlapdownUpPositionDegrees;

  public NeoSlapdownIO() {

    slapdownEncoderConfig.positionConversionFactor(360).velocityConversionFactor(6);
    // SlapDown motor
    slapdownMotorConfig = new SparkMaxConfig();
    slapdownMotorConfig.absoluteEncoder.apply(slapdownEncoderConfig);
    slapdownMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(35)
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

    slapdownMotorConfig
        .closedLoop
        .pid(
            Constants.Slapdown.kSlapdownP,
            Constants.Slapdown.kSlapdownI,
            Constants.Slapdown.kSlapdownD)
        .outputRange(-1, 1)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    SparkUtil.tryUntilOk(
        slapdownMotor,
        5,
        () ->
            slapdownMotor.configure(
                slapdownMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    slapdownPIDController = slapdownMotor.getClosedLoopController();
  }

  public void updateInputs(SlapdownIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;

    // slapdown
    SparkUtil.ifOk(
        slapdownMotor,
        slapdownEncoder::getPosition,
        (value) -> inputs.slapdownPositionDegrees = value);
    SparkUtil.ifOk(
        slapdownMotor,
        slapdownMotor::getOutputCurrent,
        (value) -> inputs.slapdownCurrentAmps = value);

    inputs.PIDSetpoint = slapdownPIDController.getSetpoint();
  }

  public void setPositionDegrees(double degrees) {
    this.setPointDegrees = degrees;
    slapdownPIDController.setSetpoint(degrees, ControlType.kPosition);
  }

  @Override
  public void setSlapdownBrakeMode(boolean enable) {
    var brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    SparkUtil.tryUntilOk(
        slapdownMotor,
        5,
        () ->
            slapdownMotor.configure(
                brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
