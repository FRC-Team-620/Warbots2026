package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);

    Logger.recordOutput("Intake/Intake Current Amps", inputs.intakeCurrentAmps);
    Logger.recordOutput("Intake/Intake Speed RPM", inputs.RPM);
    Logger.recordOutput("Intake/Slap Down Position Degrees", inputs.slapDownPositionDegrees);
    Logger.recordOutput("Intake/Slap Down Current Amps", inputs.slapDownCurrentAmps);
    Logger.recordOutput("Intake/Intake Temperature Celcius", inputs.motorTemperatureCelcius);
  }

  public void set(double speedRPM) {
    intakeIO.set(speedRPM);
  }

  public void setPosition(double degrees) {
    intakeIO.setPosition(degrees);
  }

  public void setBrakeMode(boolean enable) {
    intakeIO.setBrakeMode(enable);
  }
}
