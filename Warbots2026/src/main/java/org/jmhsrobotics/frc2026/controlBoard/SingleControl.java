package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SingleControl implements ControlBoard {
    CommandXboxController driver = new CommandXboxController(0);
    
    // ========Driver Controls========

  @Override
  public double rotation() {
    return driver.getRightX();
  }

  @Override
  public double translationX() {
    return driver.getLeftX();
  }

  @Override
  public double translationY() {
    return driver.getLeftY();
  }

}
