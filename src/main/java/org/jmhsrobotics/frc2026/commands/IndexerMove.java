package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.indexer.Indexer;

public class IndexerMove extends Command {
  private Indexer indexer;
  private double speedDutyCycle;

  public IndexerMove(Indexer indexer, double speedDutyCycle) {
    this.indexer = indexer;
    this.speedDutyCycle = speedDutyCycle;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.set(speedDutyCycle);
  }

  @Override
  public void execute() {
    indexer.set(speedDutyCycle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexer.set(0);
  }
}
