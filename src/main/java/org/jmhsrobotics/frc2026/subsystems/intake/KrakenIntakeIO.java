package org.jmhsrobotics.frc2026.subsystems.intake;

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

public class KrakenIntakeIO implements IntakeIO {
  private TalonFX intakeLeaderMotor = new TalonFX(Constants.CAN.kIntakeMotorID);
  private TalonFX intakeFollowerMotor = new TalonFX(Constants.CAN.kIntakeFollowerMotorID);

  private TalonFXConfiguration intakeLeaderConfig;
  private TalonFXConfiguration intakeFollowerConfig;

  private final StatusSignal<Current> leaderCurrent = intakeLeaderMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> leaderVelocity = intakeLeaderMotor.getVelocity();
  private final StatusSignal<Temperature> leaderTemp = intakeLeaderMotor.getDeviceTemp();

  private final StatusSignal<Current> followerCurrent = intakeFollowerMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> followerVelocity = intakeFollowerMotor.getVelocity();
  private final StatusSignal<Temperature> followerTemp = intakeFollowerMotor.getDeviceTemp();

  private double speedDutyCycle;

  public KrakenIntakeIO() {
    // TESTING: change to 30 (fact check);
    final int updatedCurrent = 20;

    // LEADER CONFIG (left)
    intakeLeaderConfig = new TalonFXConfiguration();
    intakeLeaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeLeaderConfig.CurrentLimits.StatorCurrentLimit = updatedCurrent;
    intakeLeaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // umm.. i was told i dont need to do voltage compensation
    intakeLeaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeLeaderMotor.getConfigurator().apply(intakeLeaderConfig);

    // FOLLOWER CONFIG (right)
    intakeFollowerConfig = new TalonFXConfiguration();
    intakeFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeFollowerConfig.CurrentLimits.StatorCurrentLimit = updatedCurrent;
    intakeFollowerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeFollowerMotor.getConfigurator().apply(intakeFollowerConfig);
    intakeFollowerMotor.setControl(
        new Follower(Constants.CAN.kIntakeMotorID, MotorAlignmentValue.Opposed));

    // Create the leader instance
    PhoenixUtil.tryUntilOk(5, () -> intakeLeaderMotor.getConfigurator().apply(intakeLeaderConfig));

    // Create the follower instance
    PhoenixUtil.tryUntilOk(
        5, () -> intakeFollowerMotor.getConfigurator().apply(intakeFollowerConfig));
    intakeFollowerMotor.setControl(
        new Follower(Constants.CAN.kIntakeMotorID, MotorAlignmentValue.Opposed));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    // claude pointed out smth weird that the follower values overwrote the leader's?
    // i kept it in case it was intetional

    BaseStatusSignal.refreshAll(
        leaderCurrent, leaderVelocity, leaderTemp, followerCurrent, followerVelocity, followerTemp);

    // leader values (will be overwritten below, same as original)
    inputs.intakeCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.RPM = leaderVelocity.getValueAsDouble() * 60.0; // rot/sec -> RPM
    inputs.intakeMotorTemperatureCelcius = leaderTemp.getValueAsDouble();

    // follower values overwrite leader's, matching original behavior
    inputs.intakeCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.RPM = followerVelocity.getValueAsDouble() * 60.0;
    inputs.intakeMotorTemperatureCelcius = followerTemp.getValueAsDouble();

    boolean isStalled =
        Math.abs(speedDutyCycle) > 0.1
            && Math.abs(followerVelocity.getValueAsDouble() * 60.0) < 10.0;
    inputs.stalled = isStalled;
  }

  @Override
  public void setSpeedDutyCycle(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
    intakeLeaderMotor.set(speedDutyCycle);
  }

  @Override
  public void setIntakeBrakeMode(boolean enable) {
    NeutralModeValue mode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    intakeLeaderConfig.MotorOutput.NeutralMode = mode;
    intakeFollowerConfig.MotorOutput.NeutralMode = mode;

    PhoenixUtil.tryUntilOk(5, () -> intakeLeaderMotor.getConfigurator().apply(intakeLeaderConfig));
    PhoenixUtil.tryUntilOk(
        5, () -> intakeFollowerMotor.getConfigurator().apply(intakeFollowerConfig));
  }
}
