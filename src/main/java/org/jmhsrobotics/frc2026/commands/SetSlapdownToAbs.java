package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;

public class SetSlapdownToAbs extends Command {
  private Slapdown slapdown;

  public SetSlapdownToAbs(Slapdown slapdown) {
    this.slapdown = slapdown;
    addRequirements(slapdown);
  }

  @Override
  public void initialize() {
    slapdown.setSlapdownEncoder(slapdown.getAbsPositionDegrees());
    System.out.println("Set slapdown encoder to " + slapdown.getAbsPositionDegrees() + " degrees");
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
