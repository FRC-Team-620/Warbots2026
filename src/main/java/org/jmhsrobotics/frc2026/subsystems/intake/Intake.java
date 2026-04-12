package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean isActive = false;
  private Debouncer stallDebouncer = new Debouncer(0.100, DebounceType.kBoth);
  private Alert stallAlert = new Alert("Intake Motor Stalled!", AlertType.kWarning);

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    // Trigger isStalled = new Trigger(this::isStalled);
    // isStalled.whileTrue(new IntakeMove(this, -1).withTimeout(0.5).repeatedly());
    // isStalled.onFalse(new IntakeMove(this, Constants.Intake.kSpeedDutyCycle));
  }

  public boolean isStalled() {
    return inputs.stalled;
  }

  @Override
  public void periodic() {
    stallAlert.set(stallDebouncer.calculate(inputs.stalled));
    intakeIO.updateInputs(inputs); // get output from hardware

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
