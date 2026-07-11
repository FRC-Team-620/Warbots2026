package org.jmhsrobotics.frc2026.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private TalonFXConfiguration motorConfig;

  private final StatusSignal<Current> indexerCurrent = frontLeftIndexMotor.getStatorCurrent();
  private final StatusSignal<Temperature> indexerTemp = frontLeftIndexMotor.getDeviceTemp();
  private final StatusSignal<AngularVelocity> indexerVelocity = frontLeftIndexMotor.getVelocity();

  // private double speedRPM;
  private double speedDutyCycle;

  public KrakenIndexerIO() {
    MotorAlignmentValue leftAlignment = MotorAlignmentValue.Aligned;
    MotorAlignmentValue rightAlignment = MotorAlignmentValue.Opposed;

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // TESTING: change from 5 to 20
    motorConfig.CurrentLimits.StatorCurrentLimit = 5;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    frontLeftIndexMotor.getConfigurator().apply(motorConfig);
    backLeftIndexMotor.getConfigurator().apply(motorConfig);
    frontRightIndexMotor.getConfigurator().apply(motorConfig);
    backRightIndexMotor.getConfigurator().apply(motorConfig);

    PhoenixUtil.tryUntilOk(5, () -> frontLeftIndexMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backLeftIndexMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> frontRightIndexMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backRightIndexMotor.getConfigurator().apply(motorConfig));

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
    motorConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    PhoenixUtil.tryUntilOk(5, () -> frontLeftIndexMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backLeftIndexMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> frontRightIndexMotor.getConfigurator().apply(motorConfig));
    PhoenixUtil.tryUntilOk(5, () -> backRightIndexMotor.getConfigurator().apply(motorConfig));
  }
}
