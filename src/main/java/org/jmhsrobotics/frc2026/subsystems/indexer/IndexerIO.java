package org.jmhsrobotics.frc2026.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double motorRPM;
    public double currentAMPS;
    public double motorTemperatureCelcius;
    public double outputSpeedDutyCycle;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void set(double speedDutyCycle) {}

  public default void setBrakeMode(boolean enable) {}
}
