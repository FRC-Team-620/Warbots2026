package org.jmhsrobotics.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

public class SimShooterIO implements ShooterIO {

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.currentAMPS = 0;
    inputs.velocityRPM = 0;
    inputs.voltage = 0;
    inputs.tempC = 0;
  }

  public void setRPM(double speedDutyCycle) {
    Logger.recordOutput("Shooter/speedDutyCycle", speedDutyCycle);
  }
}
