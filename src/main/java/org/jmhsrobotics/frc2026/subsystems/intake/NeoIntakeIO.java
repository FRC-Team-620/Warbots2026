package org.jmhsrobotics.frc2026.subsystems.intake;

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

public class NeoIntakeIO implements IntakeIO {
  private SparkMax intakeMotor = new SparkMax(Constants.CAN.kIntakeMotorID, MotorType.kBrushless);
  private SparkMaxConfig intakeMotorConfig;
  private SparkMax slapDownMotor =
      new SparkMax(Constants.CAN.kSlapDownMotorID, MotorType.kBrushless);
  private SparkMaxConfig slapDownMotorConfig;
  private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  private RelativeEncoder slapDownEncoder = slapDownMotor.getEncoder();
  private SparkClosedLoopController intakePIDController;
  private SparkClosedLoopController slapDownPIDController;
  private double speedRPM;
  private double angleDegrees;

  public NeoIntakeIO() {
    // intakeMotor

    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .voltageCompensation(12)
        .inverted(false)
        .closedLoop
        .pid(Constants.Intake.kIntakeP, Constants.Intake.kIntakeI, Constants.Intake.kIntakeD)
        .maxOutput(1)
        .minOutput(0)
        .maxMotion
        .cruiseVelocity(980)
        .maxAcceleration(20000);

    SparkUtil.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    intakePIDController = intakeMotor.getClosedLoopController();

    // SlapDown motor
    slapDownMotorConfig = new SparkMaxConfig();
    slapDownMotorConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(20)
        .voltageCompensation(12)
        .inverted(false)
        .closedLoop
        .pid(Constants.Intake.kSlapdownP, Constants.Intake.kSlapdownI, Constants.Intake.kSlapdownD)
        .maxOutput(1)
        .minOutput(0)
        .maxMotion
        .cruiseVelocity(980)
        .maxAcceleration(20000);

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
    SparkUtil.ifOk(intakeMotor, intakeMotor::getOutputCurrent, (value) -> inputs.intakeCurrentAmps = value);
    SparkUtil.ifOk(intakeMotor, intakeEncoder::getVelocity, (value) -> inputs.RPM = value);
    SparkUtil.ifOk(intakeMotor, intakeMotor::getMotorTemperature, (value) -> inputs.motorTemperatureCelcius = value);
    SparkUtil.ifOk(slapDownMotor, slapDownEncoder::getPosition, (value) -> inputs.slapDownPositionDegrees = value);
    SparkUtil.ifOk(slapDownMotor, slapDownMotor::getOutputCurrent, (value) -> inputs.slapDownCurrentAmps = value);

    intakePIDController.setSetpoint(this.speedRPM, ControlType.kMAXMotionVelocityControl);
  }

  @Override
  public void set(double RPM){
    this.speedRPM = RPM;
  }

  @Override
  public void setBrakeMode(boolean enable) {
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
