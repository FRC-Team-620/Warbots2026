package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;

public class ZeroSlapdownCommand extends Command {
  private Slapdown slapdown;
  private double speedDutyCycle;
  private double homingCurrentAmps;
  private double homingGoalAngle;
  private double timeoutSeconds;

  private Timer timer = new Timer();

  public ZeroSlapdownCommand(
      Slapdown slapdown,
      double timeoutSeconds,
      double homingCurrentAmps,
      double homingGoalAngle,
      double dutyCycle) {
    this.slapdown = slapdown;
    this.speedDutyCycle = dutyCycle;
    this.timeoutSeconds = timeoutSeconds;
    this.homingCurrentAmps = homingCurrentAmps;
    this.homingGoalAngle = homingGoalAngle;
    addRequirements(slapdown);
  }

  @Override
  public void initialize() {
    timer.reset();
    slapdown.setSpeedDutyCycle(speedDutyCycle);
  }

  @Override
  public void execute() {
    slapdown.setSpeedDutyCycle(speedDutyCycle);
    if (slapdown.getCurrentAmps() > homingCurrentAmps) {
      timer.start();
    } else {
      timer.reset();
    }
    if (timer.get() > timeoutSeconds) {
      slapdown.setSpeedDutyCycle(0);
      slapdown.setSlapdownEncoder(homingGoalAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() > timeoutSeconds;
  }
}
