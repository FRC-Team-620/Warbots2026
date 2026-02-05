package org.jmhsrobotics.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double currentAMPS;
    public double positionDEGREES;
    public double motorTemperatureCelcius;
    public double RPM;
  }

  public default void setPIDF(double p, double i, double d, double f) {}

  public default void setPosition(double degrees) {}

  public default void set(double rpm) {}

  public default void stop() {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setBrakeMode(boolean enable) {};
}
