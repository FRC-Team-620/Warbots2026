package org.jmhsrobotics.frc2026.subsystems.climber;

public class SimClimberIO implements ClimberIO {
  private double motorPositionCM;
  private boolean areHooksExtended;
  private double speedDutyCycle;

  public SimClimberIO() {}

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    inputs.motorPositionCM = this.motorPositionCM;
    inputs.areHooksExtended = this.areHooksExtended;
    inputs.motorSpeedDutyCycle = this.speedDutyCycle;
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
