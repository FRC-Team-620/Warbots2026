package org.jmhsrobotics.frc2026.controlBoard;

import java.util.function.DoubleSupplier;

public interface ControlBoard {
  // ========Driver Controls========
  public double rotation();

  public double translationX();

  public double translationY();

  // =======Operator Controls=======

  public DoubleSupplier shoot();
}
