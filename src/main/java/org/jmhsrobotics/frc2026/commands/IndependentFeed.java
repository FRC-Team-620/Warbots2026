package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.feeder.Feeder;

public class IndependentFeed extends Command {
  private Feeder feeder;
  private double speed;

  public IndependentFeed(Feeder feeder, double speed) {
    this.feeder = feeder;
    this.speed = speed;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    this.feeder.setFeederSpeed(0);
  }

  @Override
  public void execute() {
    this.feeder.setFeederSpeed(speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.feeder.setFeederSpeed(0);
  }
}
