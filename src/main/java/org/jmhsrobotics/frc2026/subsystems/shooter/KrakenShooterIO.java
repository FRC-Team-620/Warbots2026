package org.jmhsrobotics.frc2026.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

// all of this will be refernced off NeoShooterIO so... take that as you may
public class KrakenShooterIO implements ShooterIO {
  // public final int testRPM = 3400;

  private TalonFX leftTopMotor = new TalonFX(Constants.CAN.kLeftTopShooterMotorID);
  private TalonFX leftBottomMotor = new TalonFX(Constants.CAN.kLeftBottomShooterMotorID);
  private TalonFX rightTopMotor = new TalonFX(Constants.CAN.kRightTopShooterMotorID);
  private TalonFX rightBottomMotor = new TalonFX(Constants.CAN.kRightBottomShooterMotorID);

  private final StatusSignal<Current> leftTopStatorCurrent = leftTopMotor.getStatorCurrent();
  private final StatusSignal<AngularVelocity> leftTopVelocity = leftTopMotor.getVelocity();
  private final StatusSignal<Angle> leftTopPosition = leftTopMotor.getPosition();
  private final StatusSignal<Voltage> leftTopSupplyVoltage = leftTopMotor.getSupplyVoltage();
  private final StatusSignal<Temperature> leftTopDeviceTemp = leftTopMotor.getDeviceTemp();

  // for logging all 4, same as your old 3-wheel Logger.recordOutput calls
  private final StatusSignal<AngularVelocity> leftBottomVelocity = leftBottomMotor.getVelocity();
  private final StatusSignal<AngularVelocity> rightTopVelocity = rightTopMotor.getVelocity();
  private final StatusSignal<AngularVelocity> rightBottomVelocity = rightBottomMotor.getVelocity();

  // private TalonFXConfiguration motorConfigLeftTopLeader;
  // private TalonFXConfiguration motorConfigLeftBottonFollower;
  // private TalonFXConfiguration motorConfigRightTopFollower;
  // private TalonFXConfiguration motorConfigRightBottomFollower;

  private TalonFXConfiguration motorConfigLeftTopLeader;
  private TalonFXConfiguration followerConfig;

  /* RELATIVE ENCODERS SEEM TO BE UNNEEDED (CAN CALL .GETVELOCITY() ETC ON TALONFX OBJECT) */

  private double voltage;

  // i was told by claude that talons don't need PID controls... and that only the leader needs this
  // thing
  private final VelocityVoltage leftTopVelocityRequest = new VelocityVoltage(0);

  private double velocityRPM;
  private double goalRPM;

  public KrakenShooterIO() {
    MotorAlignmentValue leftAlignment = MotorAlignmentValue.Aligned;
    MotorAlignmentValue rightAlignment = MotorAlignmentValue.Opposed;
    // TESTING: change to 50
    final int updatedCurrent = 50;

    followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.CurrentLimits.StatorCurrentLimit = updatedCurrent;
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    leftBottomMotor.getConfigurator().apply(followerConfig);
    leftBottomMotor.setControl(new Follower(Constants.CAN.kLeftTopShooterMotorID, leftAlignment));

    rightTopMotor.getConfigurator().apply(followerConfig);
    rightTopMotor.setControl(new Follower(Constants.CAN.kLeftTopShooterMotorID, rightAlignment));

    rightBottomMotor.getConfigurator().apply(followerConfig);
    rightBottomMotor.setControl(new Follower(Constants.CAN.kLeftTopShooterMotorID, rightAlignment));

    // top left (LEADER)
    /* NOTE: THERE IS NO MIN AND MAX EQUIVALET SO WE HAVE TO MAKE SURE TO NEVER MAKE IT NEGATIVE */
    motorConfigLeftTopLeader = new TalonFXConfiguration();
    motorConfigLeftTopLeader.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // motorConfigLeftTopLeader.CurrentLimits.StatorCurrentLimit = updatedCurrent;
    // motorConfigLeftTopLeader.CurrentLimits.StatorCurrentLimitEnable = true;

    // THIS IS WHAT I CHANGE IF IT SPINS THE WRONG WAY
    motorConfigLeftTopLeader.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // CLAUDE: switched off kOnboardP/I/D/V here — those are REV SparkMax onboard-PID gains
    // (duty cycle per RPM of error) and are ~720x too small for Phoenix6 VelocityVoltage, which
    // expects volts per rotation/sec. That's why the motor wasn't spinning on the bench (commanded
    // voltage was ~0.003V at 60 RPM). Using the volts-scaled kP/kD/kV/kS/kA constants instead.
    motorConfigLeftTopLeader.Slot0.kP = Constants.ShooterConstants.kP;
    motorConfigLeftTopLeader.Slot0.kI = Constants.ShooterConstants.kI;
    motorConfigLeftTopLeader.Slot0.kD = Constants.ShooterConstants.kD;

    // feedforward
    motorConfigLeftTopLeader.Slot0.kS = Constants.ShooterConstants.kS;
    motorConfigLeftTopLeader.Slot0.kV = Constants.ShooterConstants.kV;
    motorConfigLeftTopLeader.Slot0.kA = Constants.ShooterConstants.kA;

    leftTopMotor.getConfigurator().apply(motorConfigLeftTopLeader);

    // i was told that getMotorVoltage() might need to get changed
    // also i need to call this line in robot init???
    // com.ctre.phoenix6.hardware.ParentDevice.optimizeBusUtilizationForAll(leftTopMotor,
    // leftBottomMotor, rightTopMotor, rightBottomMotor);
    leftTopMotor.getMotorVoltage().setUpdateFrequency(200);

    PhoenixUtil.tryUntilOk(5, () -> leftTopMotor.getConfigurator().apply(motorConfigLeftTopLeader));
    PhoenixUtil.tryUntilOk(5, () -> leftBottomMotor.getConfigurator().apply(followerConfig));
    PhoenixUtil.tryUntilOk(5, () -> rightTopMotor.getConfigurator().apply(followerConfig));
    PhoenixUtil.tryUntilOk(5, () -> rightBottomMotor.getConfigurator().apply(followerConfig));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // FIXME: Should make an array for each individaul input that contains values from all 3 motors,
    // then return arrays
    // SparkUtil.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.currentAMPS
    // = value);
    // SparkUtil.ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityRPM =
    // value);
    // SparkUtil.ifOk(motor, motor::getBusVoltage, (value) -> inputs.voltage =
    // value);
    // SparkUtil.ifOk(motor, motor::getMotorTemperature, (value) -> inputs.tempC =
    // value);
    // pidController.setSetpoint(this.velocityRPM,
    // ControlType.kMAXMotionVelocityControl);

    // SparkUtil.ifOk(
    //     leftFlywheelMotor,
    //     leftFlywheelMotor::getOutputCurrent,
    //     (value) -> inputs.currentAMPS = value);
    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelEncoder::getVelocity, (value) -> inputs.velocityRPM =
    // value);

    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelEncoder::getPosition, (value) -> inputs.positionROT =
    // value);
    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelMotor::getBusVoltage, (value) -> inputs.voltage = value);
    // SparkUtil.ifOk(
    //     leftFlywheelMotor, leftFlywheelMotor::getMotorTemperature, (value) -> inputs.tempC =
    // value);

    BaseStatusSignal.refreshAll(
        leftTopStatorCurrent,
        leftTopVelocity,
        leftTopPosition,
        leftTopSupplyVoltage,
        leftTopDeviceTemp,
        leftBottomVelocity,
        rightTopVelocity,
        rightBottomVelocity);

    inputs.currentAMPS = leftTopStatorCurrent.getValueAsDouble();
    inputs.velocityRPM = leftTopVelocity.getValueAsDouble() * 60.0; // rot/sec -> RPM
    inputs.positionROT = leftTopPosition.getValueAsDouble();
    inputs.voltage = leftTopSupplyVoltage.getValueAsDouble();
    inputs.tempC = leftTopDeviceTemp.getValueAsDouble();

    Logger.recordOutput("LeftTopMotorVelocity", leftTopMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "LeftBottonMotorVelcocity", leftBottomMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("RightTopMotorVelocity", rightTopMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "RightBottomMotorVelocity", rightBottomMotor.getVelocity().getValueAsDouble());

    inputs.appliedVoltage = voltage;
    inputs.goalRPM = this.goalRPM;
  }

  @Override
  public void setRPM(double velocityRPM) {
    this.goalRPM = velocityRPM;
    this.setVoltage(velocityRPM / 1000);
    // leftTopMotor.setControl(leftTopVelocityRequest.withVelocity((velocityRPM) / 60.0));
  }

  @Override
  public void setSpeed(double speed) {
    leftTopMotor.set(speed);
  }

  @Override
  public void setVoltage(double voltage) {
    leftTopMotor.setVoltage(voltage);
    this.voltage = voltage;
  }

  // ummmm @mike this was already commented out so im not touching this
  @Override
  public void setPIDF(double p, double i, double d, double f) {
    // TODO: Implement ff term
    // FIXME:
    // var setPIDF = new SparkMaxConfig();
    // setPIDF.closedLoop.pid(p, i, d);
    // SparkUtil.tryUntilOk(
    // motor,
    // 5,
    // () ->
    // motor.configure(
    // setPIDF, ResetMode.kNoResetSafeParameters,
    // PersistMode.kNoPersistParameters));
  }

  @Override
  public void stop() {
    // leftFlywheelPIDController.setSetpoint(0, ControlType.kVelocity);
    leftTopMotor.stopMotor();
  }

  // there was some freaky thing where rewriting a config completly gets rid of the other one?
  // i think it gets reestablsished in updateInputs() but what do i know
  // just mentioning it
  @Override
  public void setBrakeMode(boolean enable) {
    var brakeConfig = new TalonFXConfiguration();
    brakeConfig.MotorOutput.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    PhoenixUtil.tryUntilOk(5, () -> leftTopMotor.getConfigurator().apply(brakeConfig));
  }
}
