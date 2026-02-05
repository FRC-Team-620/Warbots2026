package org.jmhsrobotics.frc2026.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }


  @Override
  public void periodic(){
    intakeIO.updateInputs(inputs);
  

    Logger.recordOutput("Intake/Current Amps", inputs.currentAMPS);
    Logger.recordOutput("Intake/Speed RPM", inputs.RPM);
    Logger.recordOutput("Intake/Position Degrees", inputs.positionDEGREES);
    Logger.recordOutput("Intake/Temperature Celcius", inputs.motorTemperatureCelcius);
  }

  public void set(double speedRPM){
    intakeIO.set(speedRPM);
  }

  public void setBrakeMode(boolean enable){
    intakeIO.setBrakeMode(enable);
  }
}
