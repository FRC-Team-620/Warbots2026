package org.jmhsrobotics.frc2026.controlBoard;

// HOOD REMOVAL (2026-07-17): removed the hoodDown() trigger — see
// subsystems/shooter/Shooter.java for the full note.
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControlBoard {
  // ========Driver Controls========
  public double rotation();

  public Rotation2d rotationABS();

  public double translationX();

  public double translationY();

  public Trigger resetForward();

  public Trigger autoAim();

  public Trigger faceDriveDirection();

  public Trigger turbo();

  public Trigger slowdown();

  // =======Operator Controls=======

  // public Trigger shooterSpinup();
  public Trigger fieldYeet();

  public Trigger dutyCycleShoot();

  public Trigger feedAndShoot();

  public Trigger runFeeder();

  public Trigger slapdownMoveUp();

  public Trigger slapdownMoveDown();

  public Trigger indexOn();

  public Trigger intakeOn();

  public Trigger intakeOff();

  public Trigger climberUp();

  public Trigger climberDown();

  public Trigger extakeFuel();

  public Trigger ClimberExtendHooks();

  public Trigger ClimberRetractHooks();
}
