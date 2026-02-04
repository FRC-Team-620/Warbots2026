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
  FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 2, 1), DCMotor.getNEO(2));
  PIDController pid = new PIDController(1, 0, 0);

  public boolean isOpenLoop = false;
  public double outputVolts = 0;

  public SimShooterIO() {
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

    this.flywheelSim.setInput(outvolts);
    this.flywheelSim.update(Constants.ksimTimestep);

    inputs.currentAMPS = flywheelSim.getCurrentDrawAmps();
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.voltage = flywheelSim.getInputVoltage();
    inputs.tempC = 20;
  }

  @Override
  public void setRPM(double speedDutyCycle) {
    setRPM(speedDutyCycle);
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
