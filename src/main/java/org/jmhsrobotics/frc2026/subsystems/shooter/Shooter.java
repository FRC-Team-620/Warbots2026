package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private Timer accelerationTimer = new Timer();

  private boolean isActive = false;
  private double goalSpeedRPM = 0;

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);

    Logger.processInputs("/Shooter", shooterInputs);

    Logger.recordOutput("Shooter/At RPM Goal", this.atRPMGoal());

    Logger.recordOutput("Shooter/Active", isActive);

    SmartDashboard.putNumber("Shooter velocity", goalSpeedRPM);
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

    if (velocityRPM > 0) {
      isActive = true;
    } else {
      isActive = false;
    }
  }

  public void stop() {
    shooterIO.stop();
  }

  public void setBrakeMode(boolean enable) {
    shooterIO.setBrakeMode(enable);
  }

  public boolean atRPMGoal() {
    return Math.abs(shooterInputs.velocityRPM - goalSpeedRPM)
            < Constants.ShooterConstants.kShooterTolerance
        && goalSpeedRPM > 0;
  }

  public boolean isActive() {
    return isActive;
  }

  public boolean atMaxRPM() {
    // make it 100 rpm less than the max, so we get a buffer
    return this.goalSpeedRPM > Constants.ShooterConstants.kBaseRPM - 100;
  }

  public boolean notMaxRPM() {
    return this.isActive && !this.atMaxRPM();
  }
}
