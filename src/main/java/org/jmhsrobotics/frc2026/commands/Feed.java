package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.feeder.Feeder;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class Feed extends Command {
  private Feeder feeder;
  private double speed;
  private Shooter shooter;

  public Feed(Feeder feeder, double speed, Shooter shooter) {
    this.feeder = feeder;
    this.speed = speed;
    this.shooter = shooter;
    addRequirements(this.feeder);
  }

  @Override
  public void initialize() {
    this.feeder.setFeederSpeed(0);
  }

  @Override
  public void execute() {
    if (shooter.atRPMGoal()) {
      if (MathUtil.isNear(
          shooter.getServoGoal(),
          shooter.getServoPosition(),
          Constants.Shooter.kHoodTolerance)) {
        this.feeder.setFeederSpeed(this.speed);
      } else {
        this.feeder.setFeederSpeed(0);
      }
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
