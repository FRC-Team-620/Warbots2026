package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private Servo leftServo = new Servo(0);
  private Servo rightServo = new Servo(1);

  private Timer accelerationTimer = new Timer();

  private boolean isActive = false;
  private double goalSpeedRPM = 0;
  private double rpmPidOutput = 0;
  private boolean isClosedLoop = false;
  private InterpolatingDoubleTreeMap rpmMap;
  private InterpolatingDoubleTreeMap hoodMap;
  private double hoodRealPosition = 0.0;

  private double hubOffset = .584;
  private double robotOffset = .4191;
  private double totalOffset = hubOffset + robotOffset;

  private double hoodPosition = 0.31;

  public Shooter(ShooterIO shooterIO) {

    this.shooterIO = shooterIO;
    this.rpmMap = new InterpolatingDoubleTreeMap();
    createRPMMap(rpmMap);
    this.hoodMap = new InterpolatingDoubleTreeMap();
    createHoodMap(hoodMap);
    SmartDashboard.putNumber("Hood Position", hoodPosition);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    // hoodPosition = SmartDashboard.getNumber("Hood Position", 0.31);
    leftServo.setPosition(hoodPosition);
    rightServo.setPosition(hoodPosition);

    /* ----------------RPM Control------------------------ TODO: move to dedicated util method*/
    // TODO: move to constants
    // final double maxVoltageOutput = 15.0;

    // clamp the output to settable motor voltage limits
    // double clampedVoltageOutput = MathUtil.clamp(
    //     pidControllerOutput,
    //     -maxVoltageOutput,
    //     maxVoltageOutput);

    // clamp the output to settable motor percent limits

    // this.rpmPidOutput = clampedOutput;

    Logger.processInputs("Shooter", shooterInputs);

    Logger.recordOutput("Shooter/Goal RPM", shooterInputs.goalRPM);
    Logger.recordOutput("Shooter/Velocity RPM", shooterInputs.velocityRPM);
    Logger.recordOutput("Shooter/Voltage", shooterInputs.voltage);
    Logger.recordOutput("Shooter/Current", shooterInputs.currentAMPS);
    Logger.recordOutput("Shooter/Temperature C", shooterInputs.tempC);
    Logger.recordOutput("Shooter/Position", shooterInputs.positionROT);
    Logger.recordOutput("Shooter/At RPM Goal", this.atRPMGoal());

    Logger.recordOutput("Shooter/Active", isActive);

    Logger.recordOutput("Shooter/GoalSpeedRpm", goalSpeedRPM);

    Logger.recordOutput("Shooter/RPS", shooterInputs.velocityRPM / 60);

    Logger.recordOutput("Shooter/isClosedLoop", this.isClosedLoop);

    Logger.recordOutput("Shooter/Hood Position", this.hoodPosition);
  }

  public void setRPM(double velocityTargetRPM) {
    // TODO: what and why?
    Logger.recordOutput("Shooter/velocityTarget", velocityTargetRPM);
    shooterIO.setRPM(velocityTargetRPM);
    // set global RPM goal
    this.goalSpeedRPM = velocityTargetRPM;

    // // set speed to clamped calculated PID output
    // // this.setVoltage(clampedVoltageOutput);
    // this.setSpeed(this.rpmPidOutput);

    // update active state TODO: move to periodic update
    updateActiveState(this.goalSpeedRPM);
  }

  private boolean updateActiveState(double velocityRPM) {
    // if (velocityRPM > 0) {
    // isActive = true;
    // } else {
    // isActive = false;
    // }
    return isActive = velocityRPM > 0;
  }

  public void setSpeed(double speed) {
    this.isClosedLoop = false;
    if (Math.abs(speed) > 0) {
      isActive = true;
    } else {
      isActive = false;
    }
    shooterIO.setSpeed(speed);
  }

  public void setVoltage(double voltage) {
    shooterIO.setVoltage(voltage);
  }

  public void setVoltage(Voltage voltage) {
    shooterIO.setVoltage(voltage.baseUnitMagnitude());
  }

  public void setHoodPosition(double position) {
    this.hoodPosition = position;
  }

  public void stop() {
    this.goalSpeedRPM = 0.0;
    this.isClosedLoop = false;
    shooterIO.stop();
  }

  public void LogData(SysIdRoutineLog log) {
    // log.record("Shooter/Goal RPM", shooterInputs.goalRPM);
    // log.record("Shooter/Velocity RPM", shooterInputs.velocityRPM);
    // log.record("Shooter/Voltage", shooterInputs.voltage);
    // log.record("Shooter/Current", shooterInputs.currentAMPS);
    // log.record("Shooter/Temperature C", shooterInputs.tempC);
    Logger.recordOutput("Shooter/Position", shooterInputs.positionROT);
    Logger.recordOutput("Shooter/At RPM Goal", this.atRPMGoal());
  }

  public void setBrakeMode(boolean enable) {
    shooterIO.setBrakeMode(enable);
  }

  public boolean atRPMGoal() {
    return Math.abs(shooterInputs.velocityRPM - goalSpeedRPM)
            < Constants.ShooterConstants.kShooterTolerance
        && goalSpeedRPM > 0;
  }

  public double calculateEstimatedRPM(double distance) {
    return rpmMap.get(distance);
  }

  public double calculateHoodPosition(double distance) {
    return hoodMap.get(distance);
  }

  public void createRPMMap(InterpolatingDoubleTreeMap map) {
    // map.put(1.255, 3500.0);
    // map.put(4.00, 4000.0);
    map.put(1.3, 2600.0);
    // map.put(2.0, 2550.0);
    map.put(Units.inchesToMeters(37) + totalOffset, 3400.0);
  }

  public void createHoodMap(InterpolatingDoubleTreeMap map) {
    // (distance, hood height)
    // map.put(1.255, 3500.0);
    // map.put(4.00, 4000.0);
    // map.put(2.0, 0.5);
    // 16.5 in / .4191 m from edge of bumper to center of bot
    map.put(1.3, 0.31);
    // map.put(2.334, 0.31);
    map.put(Units.inchesToMeters(37) + totalOffset, 0.31);
    map.put(Units.inchesToMeters(50) + totalOffset, 0.32);
    map.put(Units.inchesToMeters(70) + totalOffset, 0.36);
  }

  public boolean isActive() {
    return isActive;
  }

  public void setServoPosition(double position) {
    leftServo.setPosition(position);
    rightServo.setPosition(position);
  }

  public double getServoPosition() {
    return leftServo.getPosition();
  }

  public double getServoGoal() {
    return hoodPosition;
  }

  public boolean atMaxRPM() {
    // make it 100 rpm less than the max, so we get a buffer
    return this.goalSpeedRPM > Constants.ShooterConstants.kBaseRPM - 100;
  }

  public boolean notMaxRPM() {
    return this.isActive && !this.atMaxRPM();
  }
}
