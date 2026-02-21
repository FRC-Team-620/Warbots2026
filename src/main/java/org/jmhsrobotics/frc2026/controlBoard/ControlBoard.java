package org.jmhsrobotics.frc2026.controlBoard;

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

  public Trigger turbo();

  // =======Operator Controls=======

  public Trigger shooterSpinup();

  public Trigger runFeeder();

  public Trigger slapdownMoveUp();

  public Trigger slapdownMoveDown();

  public Trigger indexOn();

  public Trigger intakeOn();

  public Trigger intakeOff();

  public Trigger climberUp();

  public Trigger climberDown();

  public Trigger extakeFuel();
}
