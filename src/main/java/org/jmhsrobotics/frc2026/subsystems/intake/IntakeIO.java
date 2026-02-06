package org.jmhsrobotics.frc2026.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCurrentAmps;
    public double motorTemperatureCelcius;
    public double RPM;

    public double slapDownPositionDegrees;
    public double slapDownCurrentAmps;
    public double slapDownAccelerationRPMPerSec;
    public double slapDownSpeedDegPerSec;
  }

  public default void setPIDF(double p, double i, double d, double f) {}

  public default void setPosition(double degrees) {}

  public default void set(double rpm) {}

  public default void stop() {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setBrakeMode(boolean enable, SparkMax motor) {}
  ;
}
