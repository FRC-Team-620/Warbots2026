package org.jmhsrobotics.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederSpeedDutyCycle;
    public double feederVelocityRPM;
    public double feederCurrentAMPS;
    public double feederVoltage;
    public double feederTemperatureCelcius;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setBrakeMode(boolean enable) {}

  public default void setFeederSpeed(double dutyCycle) {}

  public default void stop() {}
  ;
}
