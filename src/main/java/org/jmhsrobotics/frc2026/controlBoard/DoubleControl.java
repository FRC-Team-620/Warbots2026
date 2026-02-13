package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  public Rotation2d rotationABS() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'rotationABS'");
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
  public Trigger autoAim() {
    return driver.rightTrigger();
  }

  @Override
  public Trigger turbo() {
    return driver.leftBumper();
  }

  // ========Operator Controls========

  @Override
  public Trigger shoot() {
    return operator.rightTrigger();
  }

  @Override
  public Trigger SlapdownMoveDown() {
    return operator.povLeft();
  }

  @Override
  public Trigger SlapdownMoveUp() {
    return operator.povRight();
  }

  @Override
  public Trigger indexToggle() {
    return operator.leftBumper();
  }

  @Override
  public Trigger intakeToggle() {
    return operator.rightBumper();
  }

  @Override
  public Trigger climberUp() {
    return operator.povUp();
  }

  @Override
  public Trigger climberDown() {
    return operator.povDown();
  }

  @Override
  public Trigger extakeFuel() {
    return operator.x();
  }

  @Override
  public Trigger intakeIndexOn() {
    return operator.y();
  }
}
