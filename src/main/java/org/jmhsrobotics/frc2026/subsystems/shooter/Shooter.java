package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private Servo leftServo = new Servo(0);
  private Servo rightServo = new Servo(1);

  private PIDController rpmController = new PIDController(0.08, 0, 0.005);

  private Timer accelerationTimer = new Timer();

  private boolean isActive = false;
  private double goalSpeedRPM = 0;
  private double rpmPidOutput = 0;
  private boolean isClosedLoop = false;
  private InterpolatingDoubleTreeMap rpmMap;
  private InterpolatingDoubleTreeMap hoodMap;

  private double hoodPosition = 0.2;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    this.rpmMap = new InterpolatingDoubleTreeMap();
    createRPMMap(rpmMap);
    this.hoodMap = new InterpolatingDoubleTreeMap();
    createHoodMap(hoodMap);
    SmartDashboard.putData("ShooterFlywheel/pid", rpmController);
    SmartDashboard.putNumber("Hood Position", hoodPosition);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);

    if (isClosedLoop) {
      final double maxPercentOutput = 1;
      // calculate PID output
      double centiVeloctiyRPM = shooterInputs.velocityRPM / 100.0;
      double centiGoalSpeedRPM = this.goalSpeedRPM / 100.0;
      double pidControllerOutput =
          this.rpmController.calculate(centiVeloctiyRPM, centiGoalSpeedRPM);
      double clampedOutput =
          MathUtil.clamp(pidControllerOutput, -maxPercentOutput, maxPercentOutput);

      shooterIO.setSpeed(clampedOutput);
    }

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
    if (velocityTargetRPM > 0) {
      accelerationTimer.start();
    } else {
      accelerationTimer.reset();
      accelerationTimer.stop();
    }
    this.isClosedLoop = true;
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

  public void setHoodPosition(double position) {
    this.hoodPosition = position;
  }

  public void stop() {
    shooterIO.stop();
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
    map.put(1.3, 3000.0);
    map.put(1.85, 3300.0);
    map.put(2.1, 3500.0);
    map.put(2.55, 3650.0);
    map.put(3.0, 3700.0);
    map.put(3.5, 3670.0);
    map.put(4.0, 3790.0);
    map.put(4.5, 3870.0);
    map.put(5.0, 4000.0);
  }

  public void createHoodMap(InterpolatingDoubleTreeMap map) {
    // (distance, hood height)
    // map.put(1.255, 3500.0);
    // map.put(4.00, 4000.0);
    map.put(1.3, 0.2);
    map.put(2.1, 0.35);
    map.put(3.0, 0.5);
    map.put(4.0, 0.63);
    map.put(5.0, 0.7);
    map.put(5.5, 0.75);
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

  public boolean atMaxRPM() {
    // make it 100 rpm less than the max, so we get a buffer
    return this.goalSpeedRPM > Constants.ShooterConstants.kBaseRPM - 100;
  }

  public boolean notMaxRPM() {
    return this.isActive && !this.atMaxRPM();
  }
}
