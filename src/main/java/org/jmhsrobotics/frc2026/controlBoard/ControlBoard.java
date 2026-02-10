package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface ControlBoard {
  // ========Driver Controls========
  public double rotation();

  public Rotation2d rotationABS();

  public double translationX();

  public double translationY();

  public Trigger resetForward();

  // =======Operator Controls=======

  public DoubleSupplier shoot();

  public Trigger moveIntake();

  public Trigger SlapdownMoveUp();

  public Trigger SlapdownMoveDown();
}
