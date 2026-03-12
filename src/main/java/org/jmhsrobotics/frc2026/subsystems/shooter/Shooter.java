package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private PIDController rpmController = new PIDController(0.08, 0, 0.005);
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(0, 0); // TODO: Fill With Values From Sysid

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
    SmartDashboard.putData("ShooterFlywheel/pid", rpmController); // Add For Tuning
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);

    if (isClosedLoop) {
      // final double maxPercentOutput = 1;
      // // calculate PID output
      // double centiVeloctiyRPM = shooterInputs.velocityRPM / 100.0;
      // double centiGoalSpeedRPM = this.goalSpeedRPM / 100.0;
      // double pidControllerOutput =
      //     this.rpmController.calculate(centiVeloctiyRPM, centiGoalSpeedRPM);

      // double clampedOutput =
      //     MathUtil.clamp(pidControllerOutput, -maxPercentOutput, maxPercentOutput);
      // shooterIO.setSpeed(clampedOutput);
      double velRPS = shooterInputs.velocityRPM / 60.0;
      double goalRPS = this.goalSpeedRPM / 60.0;

      double pidControllerVoltage = this.rpmController.calculate(velRPS, goalRPS);

      double ffVolts =
          ff.calculateWithVelocities(shooterInputs.velocityRPM / 60.0, this.goalSpeedRPM / 60.0);
      shooterIO.setVoltage(ffVolts + pidControllerVoltage); // TODO May want to clamp this.
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

    Logger.recordOutput("Shooter/Goal RPM", shooterInputs.goalRPM);
    Logger.recordOutput("Shooter/Velocity RPM", shooterInputs.velocityRPM);
    Logger.recordOutput("Shooter/Voltage", shooterInputs.voltage);
    Logger.recordOutput("Shooter/Current", shooterInputs.currentAMPS);
    Logger.recordOutput("Shooter/Temperature C", shooterInputs.tempC);
    Logger.recordOutput("Shooter/Position", shooterInputs.position);
    Logger.recordOutput("Shooter/At RPM Goal", this.atRPMGoal());

    Logger.recordOutput("Shooter/Active", isActive);

    Logger.recordOutput("Shooter/GoalSpeedRpm", goalSpeedRPM);

    Logger.recordOutput("Shooter/RPS", shooterInputs.velocityRPM / 60);

    Logger.recordOutput("Shooter/isClosedLoop", this.isClosedLoop);
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

  public void setVoltage(Voltage voltage) {
    shooterIO.setVoltage(voltage.baseUnitMagnitude());
  }

  public void stop() {
    shooterIO.stop();
  }

  public void LogData(SysIdRoutineLog log) {
    // log.record("Shooter/Goal RPM", shooterInputs.goalRPM);
    // log.record("Shooter/Velocity RPM", shooterInputs.velocityRPM);
    // log.record("Shooter/Voltage", shooterInputs.voltage);
    // log.record("Shooter/Current", shooterInputs.currentAMPS);
    // log.record("Shooter/Temperature C", shooterInputs.tempC);
    Logger.recordOutput("Shooter/Position", shooterInputs.position);
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

  public boolean atMaxRPM() {
    // make it 100 rpm less than the max, so we get a buffer
    return this.goalSpeedRPM > Constants.ShooterConstants.kBaseRPM - 100;
  }

  public boolean notMaxRPM() {
    return this.isActive && !this.atMaxRPM();
  }
}
