package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2026.util.ControllerMonitor;

public class SingleControl implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);

  // TODO: also need to add controller monitor to double control when that gets made
  public SingleControl() {
    ControllerMonitor.addController(driver.getHID(), "Driver");
  }

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

  @Override
  public DoubleSupplier shoot() {
    return () -> driver.getRightTriggerAxis();
  }

  @Override
  public Trigger moveIntake() {
    return driver.y();
  }
}
