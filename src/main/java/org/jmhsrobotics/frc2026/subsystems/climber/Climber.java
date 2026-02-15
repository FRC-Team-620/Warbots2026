package org.jmhsrobotics.frc2026.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  public void periodic() {
    climberIO.updateInputs(inputs);

    Logger.processInputs("/climber", inputs);

    // TODO: Clean up. May want to change units.
    Logger.recordOutput(
        "Model/climber/right_pos",
        new Pose3d(0, 0, -inputs.motorPositionCM / 100, new Rotation3d())); // *0.30188
    Logger.recordOutput(
        "Model/climber/left_pos",
        new Pose3d(0, 0, inputs.motorPositionCM / 100, new Rotation3d())); // *0.30188
    // TODO: independent ctl of hooks?
    Logger.recordOutput(
        "Model/climber/right_hook",
        new Pose3d(
            0,
            inputs.areHooksExtended ? 0 : 0.1,
            -inputs.motorPositionCM / 100,
            new Rotation3d())); // *0.30188
    Logger.recordOutput(
        "Model/climber/left_hook",
        new Pose3d(
            0,
            inputs.areHooksExtended ? 0 : 0.1,
            inputs.motorPositionCM / 100,
            new Rotation3d())); // *0.30188
  }

  public void setPositionCM(double positionCM) {
    climberIO.setPositionCM(positionCM);
  }

  public void setBrakeMode(boolean enable) {
    climberIO.setBrakeMode(enable);
  }

  public void stop() {
    climberIO.stop();
  }

  public void retractHooks() {
    climberIO.retractHooks();
  }

  public void extendHooks() {
    climberIO.extendHooks();
  }
}
