package org.jmhsrobotics.frc2026.controlBoard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
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
  public DoubleSupplier autoAim() {
    return () -> driver.getRightTriggerAxis();
  }

  // ========Operator Controls========
  @Override
  public DoubleSupplier shoot() {
    return () -> driver.getRightTriggerAxis();
  }


  @Override
  public Trigger SlapdownMoveDown() {
    return driver.x();
  }

  @Override
  public Trigger moveIntake() {
    return driver.y();
  }

  @Override
  public Trigger SlapdownMoveUp() {
    return driver.y();
  }

  @Override
  public Rotation2d rotationABS() {
    return angle.plus(Rotation2d.fromDegrees(driver.getRightX()));
  }

  public Trigger index() {
    return driver.b();
  }
}
