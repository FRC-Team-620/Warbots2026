package org.jmhsrobotics.frc2026.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public double motorPositionCM;
    public double currentAMPS;
    public double motorTemperatureCelcius;
    public boolean areHooksExtended;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setPositionCM(double positionCM) {}

  public default void setBrakeMode(boolean enable) {}

  public default void stop() {}

  public default void retractHooks() {}

  public default void extendHooks() {}
}
