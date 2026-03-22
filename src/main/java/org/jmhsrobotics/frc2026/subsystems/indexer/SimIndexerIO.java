package org.jmhsrobotics.frc2026.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.jmhsrobotics.frc2026.Constants;

public class SimIndexerIO implements IndexerIO {

  private FlywheelSim indexerSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 3), DCMotor.getNEO(1));
  private double outputVolts;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    double outvolts = this.outputVolts;
    this.indexerSim.setInputVoltage(outvolts);
    this.indexerSim.update(Constants.ksimTimestep);

    inputs.currentAMPS = indexerSim.getCurrentDrawAmps();
    inputs.motorRPM = indexerSim.getAngularVelocityRPM();
    inputs.motorTemperatureCelcius = 20;
  }

  @Override
  public void set(double speedDutyCycle) {
    outputVolts = MathUtil.clamp(speedDutyCycle, -1, 1) * RobotController.getBatteryVoltage();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // TODO implement in sim
  }
}
