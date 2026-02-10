package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private Timer accelerationTimer = new Timer();

  private boolean isAccelerating;

  private double goalSpeedRPM;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);

    Logger.processInputs("/shooter", shooterInputs);

    Logger.recordOutput("Shooter/Shooter Current Amps", shooterInputs.currentAMPS);
    Logger.recordOutput("Shooter/Shooter Voltage", shooterInputs.voltage);
    Logger.recordOutput("Shooter/Shooter Goal RPM", shooterInputs.goalRPM);
    Logger.recordOutput("Shooter/Shooter RPM", shooterInputs.velocityRPM);
    Logger.recordOutput("Shooter/Shooter Motor Temperature", shooterInputs.tempC);
    Logger.recordOutput("Shooter/At RPM Goal", this.atRPMGoal());
  }

  public void setRPM(double velocityRPM) {
    if (velocityRPM > 0) {
      accelerationTimer.start();
    } else {
      accelerationTimer.reset();
      accelerationTimer.stop();
    }
    goalSpeedRPM = velocityRPM;
    shooterIO.setRPM(velocityRPM);
  }

  public void stop() {
    shooterIO.stop();
  }

  public void setBrakeMode(boolean enable) {
    shooterIO.setBrakeMode(enable);
  }

  private boolean atRPMGoal() {
    return Math.abs(shooterInputs.velocityRPM - goalSpeedRPM) < 100;
  }
}
