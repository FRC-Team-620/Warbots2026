package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.feeder.Feeder;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

/* This command is runs the feeder without needing the shooter to be at its goal rpm. It is meant to be used with duty cycle shoot */

public class IndependentFeed extends Command {
  private Feeder feeder;
  private double dutyCycle;
  private Shooter shooter;

  public IndependentFeed(Feeder feeder, double dutyCycle, Shooter shooter) {
    this.feeder = feeder;
    this.dutyCycle = dutyCycle;
    this.shooter = shooter;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (this.shooter.isActive()) {
      this.feeder.setFeederSpeed(dutyCycle);
    } else {
      this.feeder.setFeederSpeed(0);
    }
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
