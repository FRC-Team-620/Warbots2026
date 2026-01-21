package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private ShooterIO shooterIO;
  private ShooterIO.ShooterIOInputs shooterInputs = new ShooterIO.ShooterIOInputs();

  // TODO makeAutoLogged

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.recordOutput("Shooter/Shooter Current Amps", shooterInputs.CurrentAMPS);
    Logger.recordOutput("Shooter/Shooter Voltage", shooterInputs.Voltage);
    Logger.recordOutput("Shooter/Shooter Goal RPM", shooterInputs.GoalRPM);
    Logger.recordOutput("Shooter/Shooter RPM", shooterInputs.RPM);
    Logger.recordOutput("Shooter/Shooter Motor Temperature", shooterInputs.TEMP);
  }

  public void set(double speedDutyCycle) {
    shooterIO.setRPM(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    shooterIO.setBrakeMode(enable);
  }
}
