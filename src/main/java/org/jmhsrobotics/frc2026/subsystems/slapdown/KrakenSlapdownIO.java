package org.jmhsrobotics.frc2026.subsystems.slapdown;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.slapdown.SlapdownIO.SlapdownIOInputs;
import org.jmhsrobotics.frc2026.util.PhoenixUtil;
import org.jmhsrobotics.frc2026.util.SparkUtil;

public class KrakenSlapdownIO implements SlapdownIO {
  private TalonFX slapdownKrakenMotor = new TalonFX(Constants.CAN.kSlapdownMotorID);
  private TalonFXConfiguration slapdownKrakenMotorConfig;
  private FeedbackConfigs slapdownKrakenFeedbackConfig = new FeedbackConfigs();

  private final StatusSignal<Current> slapdownCurrent = slapdownKrakenMotor.getStatorCurrent();
  private final StatusSignal<Temperature> slapdownTemp = slapdownKrakenMotor.getDeviceTemp();
  private final StatusSignal<AngularVelocity> slapdownVelocity = slapdownKrakenMotor.getVelocity();
  private final StatusSignal<Angle> slapdownPosition = slapdownKrakenMotor.getPosition();

  private double setPointDegrees;
  private final PositionVoltage positionRequest = new PositionVoltage(0);

  public KrakenSlapdownIO() {
    // semi-verified by ishaan 7/11/26
    // slapdown is 20:1 ratio

    // TESTING: change to 35 (fact check)
    final int updatedCurrent = 35;

    slapdownKrakenMotorConfig = new TalonFXConfiguration();
    slapdownKrakenMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    slapdownKrakenMotorConfig.CurrentLimits.StatorCurrentLimit = updatedCurrent;
    slapdownKrakenMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    slapdownKrakenMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // slapdownKrakenFeedbackConfig.RotorToSensorRatio = 1.0;
    slapdownKrakenFeedbackConfig.SensorToMechanismRatio = 20.0;
    slapdownKrakenMotorConfig.withFeedback(slapdownKrakenFeedbackConfig);

    slapdownKrakenMotorConfig.Slot0.kP = Constants.Slapdown.kSlapdownP;
    slapdownKrakenMotorConfig.Slot0.kI = Constants.Slapdown.kSlapdownI;
    slapdownKrakenMotorConfig.Slot0.kD = Constants.Slapdown.kSlapdownD;

    slapdownKrakenMotor.getConfigurator().apply(slapdownKrakenMotorConfig);

    PhoenixUtil.tryUntilOk(
        5, () -> slapdownKrakenMotor.getConfigurator().apply(slapdownKrakenMotorConfig));
  }

  public void updateInputs(SlapdownIOInputs inputs) {
    SparkUtil.sparkStickyFault = false;

    BaseStatusSignal.refreshAll(slapdownCurrent, slapdownTemp, slapdownVelocity, slapdownPosition);

    inputs.slapdownCurrentAmps = slapdownCurrent.getValueAsDouble();
    // inputs.motorTemperatureCelcius = slapdownTemp.getValueAsDouble();
    inputs.slapdownSpeedDegPerSec =
        slapdownVelocity.getValueAsDouble() * 360.0; // rot/sec -> deg/sec

    // TODO fix back to abs position
    inputs.slapdownAbsPositionDegrees = slapdownPosition.getValueAsDouble() * 360.0; // rot -> deg
    inputs.PIDSetpoint = this.setPointDegrees;
  }

  public void setSlapdownEncoder(double positionDegrees) {
    slapdownKrakenMotor.setPosition(positionDegrees / 360.0); // degrees -> rotations
  }

  public void setPositionDegrees(double degrees) {
    this.setPointDegrees = degrees;
    slapdownKrakenMotor.setControl(
        positionRequest.withPosition(degrees / 360.0)); // deg -> rotations
  }

  public void setSpeedDutyCycle(double dutyCycle) {
    slapdownKrakenMotor.set(dutyCycle);
  }

  @Override
  public void setSlapdownBrakeMode(boolean enable) {
    slapdownKrakenMotorConfig.MotorOutput.NeutralMode =
        enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(
        5, () -> slapdownKrakenMotor.getConfigurator().apply(slapdownKrakenMotorConfig));
  }

  @Override
  public void setPID(double p, double i, double d) {
    slapdownKrakenMotorConfig.Slot0.kP = p;
    slapdownKrakenMotorConfig.Slot0.kI = i;
    slapdownKrakenMotorConfig.Slot0.kD = d;
    PhoenixUtil.tryUntilOk(
        5, () -> slapdownKrakenMotor.getConfigurator().apply(slapdownKrakenMotorConfig));
  }
}
