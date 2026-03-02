package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private PIDController rpmController = new PIDController(0, 0, 0);

  private Timer accelerationTimer = new Timer();

  private boolean isActive = false;
  private double goalSpeedRPM = 0;
  private double rpmPidOutput = 0;
  private boolean isClosedLoop = false;
  private InterpolatingDoubleTreeMap map;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    this.map = new InterpolatingDoubleTreeMap();
    createInterpolatingDoubleTreeMap(map);
    SmartDashboard.putData("ShooterFlywheel/pid", rpmController);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);

    if (isClosedLoop) {
      final double maxPercentOutput = 1;
      // calculate PID output
      double centiVeloctiyRPM = shooterInputs.velocityRPM / 100.0;
      double centiGoalSpeedRPM = this.goalSpeedRPM / 100.0;
      double pidControllerOutput = this.rpmController.calculate(centiVeloctiyRPM);
      double clampedOutput =
          MathUtil.clamp(pidControllerOutput, -maxPercentOutput, maxPercentOutput);

      shooterIO.setSpeed(clampedOutput);
    }

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
    return map.get(distance);
  }

  public void createInterpolatingDoubleTreeMap(InterpolatingDoubleTreeMap map) {
    // map.put(1.255, 3500.0);
    // map.put(4.00, 4000.0);
    map.put(2.0, 3200.0);
    map.put(3.148, 4100.0);
    map.put(1.25, 2900.0);
  }

  public boolean isActive() {
    return isActive;
  }
}
