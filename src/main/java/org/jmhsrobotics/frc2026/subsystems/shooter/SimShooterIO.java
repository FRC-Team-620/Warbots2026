package org.jmhsrobotics.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class SimShooterIO implements ShooterIO {

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.CurrentAMPS = 0;
    inputs.RPM = 0;
    inputs.Voltage = 0;
    inputs.TEMP = 0;
  }

  public void setRPM(double speedDutyCycle) {
    Logger.recordOutput("Shooter/speedDutyCycle", speedDutyCycle);
  }
}
