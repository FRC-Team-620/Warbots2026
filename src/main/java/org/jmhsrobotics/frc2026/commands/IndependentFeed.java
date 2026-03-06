package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.feeder.Feeder;

/* This command is runs the feeder without needing the shooter to be at its goal rpm. It is meant to be used with duty cycle shoot */

public class IndependentFeed extends Command {
  private Feeder feeder;
  private double dutyCycle;

  public IndependentFeed(Feeder feeder, double dutyCycle) {
    this.feeder = feeder;
    this.dutyCycle = dutyCycle;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    this.feeder.setFeederSpeed(dutyCycle);
  }

  @Override
  public void execute() {
    this.feeder.setFeederSpeed(dutyCycle);
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
