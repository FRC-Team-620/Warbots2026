package org.jmhsrobotics.frc2026.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.PhoenixUtil;

public class KrakenIndexerIO implements IndexerIO {
  private TalonFX frontLeftIndexMotor = new TalonFX(Constants.CAN.kFrontLeftIndexerMotorID);
  private TalonFX backLeftIndexMotor = new TalonFX(Constants.CAN.kBackLeftIndexerMotorID);
  private TalonFX frontRightIndexMotor = new TalonFX(Constants.CAN.kFrontRightIndexerMotorID);
  private TalonFX backRightIndexMotor = new TalonFX(Constants.CAN.kBackRightIndexerMotorID);

  private TalonFXConfiguration followMotorConfig;
  private TalonFXConfiguration leadMotorConfig;

  private final StatusSignal<Current> indexerCurrent = frontLeftIndexMotor.getStatorCurrent();
  private final StatusSignal<Temperature> indexerTemp = frontLeftIndexMotor.getDeviceTemp();
  private final StatusSignal<AngularVelocity> indexerVelocity = frontLeftIndexMotor.getVelocity();

  // private double speedRPM;
  private double speedDutyCycle;

  public KrakenIndexerIO() {
    MotorAlignmentValue leftAlignment = MotorAlignmentValue.Aligned;
    MotorAlignmentValue rightAlignment = MotorAlignmentValue.Opposed;

    //lead motor (front left)
    leadMotorConfig = new TalonFXConfiguration();
    leadMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TESTING: change from 5 to 20
    leadMotorConfig.CurrentLimits.StatorCurrentLimit = 5;
    leadMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leadMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontLeftIndexMotor.getConfigurator().apply(leadMotorConfig);

    //follower config (everything else)
    followMotorConfig = new TalonFXConfiguration();
    followMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TESTING: change from 5 to 20
    followMotorConfig.CurrentLimits.StatorCurrentLimit = 5;
    followMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    backLeftIndexMotor.getConfigurator().apply(followMotorConfig);
    frontRightIndexMotor.getConfigurator().apply(followMotorConfig);
    backRightIndexMotor.getConfigurator().apply(followMotorConfig);

    PhoenixUtil.tryUntilOk(5, () -> frontLeftIndexMotor.getConfigurator().apply(leadMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backLeftIndexMotor.getConfigurator().apply(followMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> frontRightIndexMotor.getConfigurator().apply(followMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backRightIndexMotor.getConfigurator().apply(followMotorConfig));
    
    backLeftIndexMotor.setControl(
        new Follower(Constants.CAN.kFrontLeftIndexerMotorID, leftAlignment));
    frontRightIndexMotor.setControl(
        new Follower(Constants.CAN.kFrontLeftIndexerMotorID, rightAlignment));
    backRightIndexMotor.setControl(
        new Follower(Constants.CAN.kFrontLeftIndexerMotorID, rightAlignment));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(indexerCurrent, indexerTemp, indexerVelocity);

    inputs.currentAMPS = indexerCurrent.getValueAsDouble();
    inputs.motorTemperatureCelcius = indexerTemp.getValueAsDouble();
    inputs.motorRPM = indexerVelocity.getValueAsDouble() * 60.0; // rot/sec -> RPM
    inputs.outputSpeedDutyCycle = this.speedDutyCycle;
  }

  public void set(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    frontLeftIndexMotor.set(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    followMotorConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leadMotorConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> frontLeftIndexMotor.getConfigurator().apply(leadMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backLeftIndexMotor.getConfigurator().apply(followMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> frontRightIndexMotor.getConfigurator().apply(followMotorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backRightIndexMotor.getConfigurator().apply(followMotorConfig));
  }
}
