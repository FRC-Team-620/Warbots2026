package org.jmhsrobotics.frc2026.subsystems.climber;

public class SimClimberIO implements ClimberIO {
  private double motorPositionCM;
  private boolean areHooksExtended;

  public SimClimberIO() {}

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    inputs.motorPositionCM = this.motorPositionCM;
    inputs.areHooksExtended = this.areHooksExtended;
  }

  public void setPositionCM(double positionCM) {
    this.motorPositionCM = positionCM;
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
