package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface ControlBoard {
  // ========Driver Controls========
  public double rotation();

  public double translationX();

  public double translationY();

  // =======Operator Controls=======

  public DoubleSupplier shoot();

  public Trigger SlapdownMoveUp();

  public Trigger SlapdownMoveDown();
}
