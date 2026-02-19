package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jmhsrobotics.frc2026.Constants;

public class SimShooterIO implements ShooterIO {
  public static final double IN_TO_KG_MOI = 0.0002926397; // TODO: Move to constants
  public static final double MOI = 19.479 * IN_TO_KG_MOI; // TODO add units to var name
  public static final DCMotor MOTOR = DCMotor.getNEO(3);
  public static final double GEEARING = 1;

  public static final DCMotor FEEDER_MOTOR = DCMotor.getNEO(1);
  public static final double FEEDER_MOI = 0.1; // TODO: Find actual MOI of feeder wheel

  FlywheelSim flywheelSim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(MOTOR, MOI, GEEARING), MOTOR);

  FlywheelSim feederSim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(FEEDER_MOTOR, FEEDER_MOI, 1), FEEDER_MOTOR);
  PIDController pid;
  public boolean isOpenLoop = false;
  public double outputVolts = 0;
  public double feederSpeedDutyCycle = 0;
  public double feederOutputVolts = 0;

  public SimShooterIO(
      double k, double i, double d) { // TODO: This should prob be stored elsewhere (pid gains)
    pid = new PIDController(k, i, d);

    SmartDashboard.putData("sim/flywheel/pid", pid);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double outvolts = this.outputVolts;
    if (!isOpenLoop) {
      outvolts =
          MathUtil.clamp(
              this.pid.calculate(flywheelSim.getAngularVelocityRPM()),
              -RobotController.getBatteryVoltage(),
              RobotController.getBatteryVoltage());
    }
    this.flywheelSim.setInputVoltage(outvolts);
    this.flywheelSim.update(Constants.ksimTimestep);

    double feederOutputVolts = this.feederSpeedDutyCycle * RobotController.getBatteryVoltage();
    this.feederSim.setInputVoltage(feederOutputVolts);
    this.feederSim.update(Constants.ksimTimestep);

    inputs.currentAMPS = flywheelSim.getCurrentDrawAmps();
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.voltage = flywheelSim.getInputVoltage();
    inputs.tempC = 20;


    inputs.feederCurrentAMPS = feederSim.getCurrentDrawAmps();
    inputs.feederSpeedDutyCycle = this.feederSpeedDutyCycle;
    inputs.feederVoltage = feederSim.getInputVoltage();
    inputs.feederTemperatureCelcius = 20;
  }

  @Override
  public void setRPM(double RPM) {
    isOpenLoop = false;
    pid.setSetpoint(RPM);
  }

  public void setFeederSpeed(double dutyCycle) {
    this.feederSpeedDutyCycle = dutyCycle;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // TODO: Implement in SIm
  }

  @Override
  public void setPIDF(double p, double i, double d, double f) {
    pid.setPID(p, i, d);
    // TODO: Implement Feed forward

  }

  @Override
  public void stop() {
    // Set motor to open loop mode and set output voltage to 0
    this.isOpenLoop = true;
    this.outputVolts = 0;
  }
}
