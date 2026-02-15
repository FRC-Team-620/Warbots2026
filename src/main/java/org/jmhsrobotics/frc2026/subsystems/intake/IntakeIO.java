package org.jmhsrobotics.frc2026.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

// TODO: in future we may want to split up this hardware layer for arm and intake motors
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeCurrentAmps;
    public double intakeMotorTemperatureCelcius;
    public double RPM;
    public double outputSpeedDutyCycle;

    public double slapDownPositionDegrees;
    public double slapDownCurrentAmps;
    public double slapDownSpeedDegPerSec;
    public double PIDSetpoint;
  }

  public default void setPIDF(
      double p,
      double i,
      double d,
      double f) {} // TODO: inclear what loop this PIDF is setting, is it ment for the arm or intake

  // motors
  

  /**
   * Sets the Position of the slapdown intake arm
   * @param degrees
   */
  public default void setPositionDegrees(double degrees) {}

  /**
   * Sets the speed of the intake Rollers +1 is intake full speed. -1 is expell
   * @param dutyCycle
   */
  public default void setIntakeSpeed(double dutyCycle) {}

  public default void stop() {}

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeBrakeMode(boolean enable) {}

  public default void setSlapDownBrakeMode(boolean enable) {}
  ;
}
