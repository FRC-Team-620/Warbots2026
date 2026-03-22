package org.jmhsrobotics.frc2026.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.jmhsrobotics.frc2026.Constants;

public class SimClimberIO implements ClimberIO {
  private double motorPositionCM;
  private boolean areHooksExtended;
  private double speedDutyCycle;
  private ElevatorSim climberSim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          25,
          Units.lbsToKilograms(1),
          Units.inchesToMeters(1),
          0,
          Units.inchesToMeters(20),
          false,
          0);

  // private FlywheelSim climberSim = new
  // FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.02, 25),
  // DCMotor.getNEO(1));
  public SimClimberIO() {}

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    inputs.motorPositionCM = this.motorPositionCM;
    inputs.areHooksExtended = this.areHooksExtended;
    climberSim.setInputVoltage(
        MathUtil.clamp(this.speedDutyCycle, -1, 1) * RobotController.getBatteryVoltage());
    climberSim.update(Constants.ksimTimestep);

    inputs.motorSpeedDutyCycle = this.speedDutyCycle;
    inputs.currentAMPS = climberSim.getCurrentDrawAmps();
    inputs.motorPositionCM = climberSim.getPositionMeters() * 100.0;
  }

  public void setPositionCM(double positionCM) {
    this.motorPositionCM = positionCM;
  }

  public void setSpeedDutyCycle(double speedDutyCycle) {
    this.speedDutyCycle = speedDutyCycle;
  }

  public void setBrakeMode(boolean enable) {}

  public void stop() {}

  public void retractHooks() {
    this.areHooksExtended = false;
  }

  public void extendHooks() {
    this.areHooksExtended = true;
  }
}
