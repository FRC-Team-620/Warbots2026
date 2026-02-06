package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.recordOutput("Shooter/Shooter Current Amps", shooterInputs.currentAMPS);
    Logger.recordOutput("Shooter/Shooter Voltage", shooterInputs.voltage);
    Logger.recordOutput("Shooter/Shooter Goal RPM", shooterInputs.goalRPM);
    Logger.recordOutput("Shooter/Shooter RPM", shooterInputs.velocityRPM);
    Logger.recordOutput("Shooter/Shooter Motor Temperature", shooterInputs.tempC);
  }

  public void set(double velocityRPM) {
    shooterIO.set(velocityRPM);
  }

  public void stop() {
    shooterIO.stop();
  }

  public void setBrakeMode(boolean enable) {
    shooterIO.setBrakeMode(enable);
  }
}
