package org.jmhsrobotics.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double velocityRPM;
    public double goalRPM;
    public double voltage;
    public double currentAMPS;
    public double tempC;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setRPM(double rpm) {}

  public default void setPIDF(double p, double i, double d, double f) {}

  public default void setBrakeMode(boolean enable) {}

  public default void setFeederSpeed(double dutyCycle) {}

  public default void stop() {}
  ;
}
