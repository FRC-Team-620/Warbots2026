package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;

public class SlapdownJiggle extends Command {
  private Slapdown slapdown;

  public SlapdownJiggle(Slapdown slapdown) {
    this.slapdown = slapdown;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double position = slapdown.getPositionDegrees();
    if (position >= Constants.Slapdown.kSlapdownJiggleDownDegrees) {
      slapdown.setPositionDegrees(Constants.Slapdown.kSlapdownJiggleUpDegrees);
    } else if (position <= Constants.Slapdown.kSlapdownJiggleUpDegrees) {
      slapdown.setPositionDegrees(Constants.Slapdown.kSlapdownJiggleDownDegrees);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    slapdown.setPositionDegrees(Constants.Slapdown.kSlapdownDownPositionDegrees);
  }
}
