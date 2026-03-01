package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2026.util.ControllerMonitor;

public class SingleControl implements ControlBoard {
  CommandXboxController driver = new CommandXboxController(0);
  private Rotation2d angle = new Rotation2d();

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
  public Rotation2d rotationABS() {
    return angle.plus(Rotation2d.fromDegrees(driver.getRightX()));
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
  public Trigger shooterSpinup() {
    return driver.rightTrigger();
  }

  @Override
  public Trigger runFeeder() {
    return driver.leftTrigger();
  }

  @Override
  public Trigger slapdownMoveDown() {
    return driver.povLeft();
  }

  @Override
  public Trigger slapdownMoveUp() {
    return driver.povRight();
  }

  @Override
  public Trigger indexOn() {
    return driver.leftBumper();
  }

  @Override
  public Trigger intakeOn() {
    return driver.b();
  }

  @Override
  public Trigger intakeOff() {
    return driver.a();
  }

  @Override
  public Trigger climberUp() {
    return driver.povUp();
  }

  @Override
  public Trigger climberDown() {
    return driver.povDown();
  }

  @Override
  public Trigger extakeFuel() {
    return driver.x();
  }
}
