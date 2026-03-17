package org.jmhsrobotics.frc2026.subsystems.slapdown;

import org.littletonrobotics.junction.AutoLog;

public interface SlapdownIO {
  @AutoLog
  public static class SlapdownIOInputs {
    public double slapdownPositionDegrees;
    public double slapdownCurrentAmps;
    public double slapdownSpeedDegPerSec;
    public double PIDSetpoint;
    public double slapdownAbsPositionDegrees;
    public double primaryEncoderPos;
    public double absoluteEncoderPos;
  }

  public default void setPositionDegrees(double degrees) {}

  public default void setSlapdownBrakeMode(boolean enable) {}

  public default void setPID(double p, double i, double d) {}

  public default void updateInputs(SlapdownIOInputs inputs) {}

  public default void setSpeedDutyCycle(double dutyCycle) {}

  public default void setSlapdownEncoder(double positionDegrees) {}
}
