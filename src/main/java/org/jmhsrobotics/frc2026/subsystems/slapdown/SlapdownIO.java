package org.jmhsrobotics.frc2026.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
  @AutoLog
  public static class SlapdownIOInputs {
    public double slapdownPositionDegrees;
    public double slapdownCurrentAmps;
    public double slapdownSpeedDegPerSec;
    public double PIDSetpoint;
  }

  public default void setPositionDegrees(double degrees) {}

  public default void setSlapdownBrakeMode(boolean enable) {}

  public default void updateInputs(SlapdownIOInputs inputs) {}
}
