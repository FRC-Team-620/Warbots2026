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
    // slapdown is 40:1 ratio

    slapdownEncoderConfig.positionConversionFactor(360).velocityConversionFactor(6);
    // SlapDown motor
    slapdownMotorConfig = new SparkMaxConfig();
    // Convert motor rotations to arm degrees. With a 40:1 gearbox, each motor
    // rotation equals 1/40 of an arm rotation, so degrees per motor rotation = 360/40 = 9.0
    slapdownMotorConfig.encoder.positionConversionFactor(360.0 / 40.0);
    // Convert motor velocity (RPM) to degrees per second to match the absolute encoder's
    // velocity units. Relative encoder velocity is in RPM by default, so multiply by
    // (degrees per rotation) / 60.0 -> (360/40) / 60 = 0.15
    slapdownMotorConfig.encoder.velocityConversionFactor(360.0 / 40.0 / 60.0);
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
        .outputRange(-0.5, 0.3)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    SparkUtil.tryUntilOk(
        slapdownMotor,
        5,
        () ->
            slapdownMotor.configure(
                slapdownMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    slapdownMotor.getEncoder().setPosition(slapdownMotor.getAbsoluteEncoder().getPosition());
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
    SparkUtil.ifOk(
        slapdownMotor,
        slapdownMotor.getAbsoluteEncoder()::getPosition,
        (value) -> inputs.absoluteEncoderPos = value);
    SparkUtil.ifOk(
        slapdownMotor, slapdownEncoder::getPosition, (value) -> inputs.primaryEncoderPos = value);

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

  @Override
  public void setPID(double p, double i, double d) {
    var setPID = new SparkMaxConfig();
    setPID.closedLoop.pid(p, i, d);
    SparkUtil.tryUntilOk(
        slapdownMotor,
        5,
        () ->
            slapdownMotor.configure(
                setPID, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
