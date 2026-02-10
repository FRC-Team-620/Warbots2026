package org.jmhsrobotics.frc2026.subsystems.climber;

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
