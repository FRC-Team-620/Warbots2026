package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.jmhsrobotics.frc2026.util.ControllerMonitor;

public class DoubleControl implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  public DoubleControl() {
    ControllerMonitor.addController(driver.getHID(), "Driver");
    ControllerMonitor.addController(operator.getHID(), "Operator");
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
  public Trigger resetForward() {
    return driver.rightBumper();
  }

  @Override
  public double autoAim() {
    return driver.getRightTriggerAxis();
  }

  // ========Operator Controls========

  @Override
  public double shoot() {
    return operator.getRightTriggerAxis();
  }

  @Override
  public Trigger SlapdownMoveDown() {
    return operator.x();
  }

  @Override
  public Trigger SlapdownMoveUp() {
    return operator.y();
  }

  @Override
  public Trigger moveIntake() {
    return operator.y();
  }

  public Trigger index() {
    return operator.b();
  }

  @Override
  public Rotation2d rotationABS() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'rotationABS'");
  }
}
