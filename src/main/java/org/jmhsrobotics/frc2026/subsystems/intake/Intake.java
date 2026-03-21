package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2026.commands.IntakeMove;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean isActive = false;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    Trigger isStalled = new Trigger(this::isStalled);
    isStalled.onTrue(new IntakeMove(this, -1).withTimeout(0.25));
  }

  public boolean isStalled() {
    return inputs.stalled;
  }

  @Override
  public void periodic() {

    intakeIO.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/Intake Current Amps", inputs.intakeCurrentAmps);
    Logger.recordOutput("Intake/Intake Speed RPM", inputs.RPM);
    Logger.recordOutput("Intake/Intake Temperature Celcius", inputs.intakeMotorTemperatureCelcius);
    Logger.recordOutput("Intake/isActive", isActive);
  }

  public void set(double speedDutyCycle) {
    intakeIO.setSpeedDutyCycle(speedDutyCycle);
    this.setIsActive(speedDutyCycle > 0);
  }

  public void setIntakeBrakeMode(boolean enable) {
    intakeIO.setIntakeBrakeMode(enable);
  }

  public boolean getIsActive() {
    return isActive;
  }

  private void setIsActive(boolean isActive) {
    this.isActive = isActive;
  }
}
