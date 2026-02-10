package org.jmhsrobotics.frc2026.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import org.littletonrobotics.junction.AutoLog;

// TODO: in future we may want to split up this hardware layer for arm and intake motors
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCurrentAmps;
    public double motorTemperatureCelcius; // TODO: What motor is this?
    public double RPM;

    public double slapDownPositionDegrees;
    public double slapDownCurrentAmps;
    public double slapDownAccelerationRPMPerSec; // TODO: Why do we have this unit?
    public double slapDownSpeedDegPerSec;
  }

  public default void setPIDF(
      double p,
      double i,
      double d,
      double f) {} // TODO: inclear what loop this PIDF is setting, is it ment for the arm or intake

  // motors

  public default void setPosition(double degrees) {}

  public default void setRPM(
      double rpm) {} // TODO: we may want to stick with open loop control for intake

  public default void stop() {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setBrakeMode(
      boolean enable,
      SparkMax
          motor) {} // FIXME: hardware should not be passed via an IO interface. REMOVE SPARKMAX
  // from this method.
  ;
}
