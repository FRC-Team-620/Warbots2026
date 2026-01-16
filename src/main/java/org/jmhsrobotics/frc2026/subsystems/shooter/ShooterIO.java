package org.jmhsrobotics.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double RPM;
    public double GoalRPM;
    public double Voltage;
    public double CurrentAMPS;
    public double TEMP;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setRPM(double theRPM) {}

  public default void setPIDF(double F) {}
}
