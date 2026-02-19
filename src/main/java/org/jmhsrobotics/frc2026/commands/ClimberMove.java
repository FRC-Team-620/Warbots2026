package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.climber.Climber;

public class ClimberMove extends Command {
  private Climber climber;
  private double speedDutyCycle;
  private Timer timer = new Timer();

  public ClimberMove(Climber climber, double speedDutyCycle) {
    this.climber = climber;
    this.speedDutyCycle = speedDutyCycle;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    timer.reset();
    climber.setSpeedDutyCycle(speedDutyCycle);
  }

  @Override
  public void execute() {
    climber.setSpeedDutyCycle(speedDutyCycle);
    if (climber.getCurrentAmps() > 40) {
      timer.start();
    } else {
      timer.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return timer.get() > 0.25;
  }
}
