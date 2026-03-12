package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;

public class ZeroSlapdownCommand extends Command {
  private Slapdown slapdown;
  private double speedDutyCycle = 0.3;
  private Timer timer = new Timer();

  public ZeroSlapdownCommand(Slapdown slapdown) {
    this.slapdown = slapdown;
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
    if (slapdown.getCurrentAmps() > 40) {
      timer.start();
    } else {
      timer.reset();
    }
    if (timer.get() > 0.30) {
      slapdown.setSpeedDutyCycle(0);
      slapdown.setSlapdownEncoder(60);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 0.30;
  }
}
