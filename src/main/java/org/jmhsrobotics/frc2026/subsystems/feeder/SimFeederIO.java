package org.jmhsrobotics.frc2026.subsystems.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.jmhsrobotics.frc2026.Constants;

public class SimFeederIO implements FeederIO {
  private double feederSpeedDutyCycle = 0;
  private double feederOutputVolts = 0;

  public static final DCMotor FEEDER_MOTOR = DCMotor.getNEO(1);
  public static final double FEEDER_MOI = 0.1; // TODO: Find actual MOI of feeder wheel

  FlywheelSim feederSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(FEEDER_MOTOR, FEEDER_MOI, 1), FEEDER_MOTOR);
  ; // TODO: Initialize with proper parameters

  public void updateInputs(FeederIO.FeederIOInputs inputs) {
    inputs.feederCurrentAMPS = feederSim.getCurrentDrawAmps();
    inputs.feederSpeedDutyCycle = this.feederSpeedDutyCycle;
    inputs.feederVoltage = feederSim.getInputVoltage();
    inputs.feederTemperatureCelcius = 20;

    feederOutputVolts = this.feederSpeedDutyCycle * RobotController.getBatteryVoltage();
    this.feederSim.setInputVoltage(feederOutputVolts);
    this.feederSim.update(Constants.ksimTimestep);
  }

  public void setFeederSpeed(double dutyCycle) {
    this.feederSpeedDutyCycle = dutyCycle;
  }
}
