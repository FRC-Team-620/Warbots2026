package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged shooterInputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {}
}