package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;

public class SlapdownJiggle extends Command {
  private Slapdown slapdown;
  public Timer time = new Timer();

  public SlapdownJiggle(Slapdown slapdown) {
    this.slapdown = slapdown;
  }

  @Override
  public void initialize() {
    this.time.reset();
    this.time.start();
  }

  @Override
  public void execute() {
    double position = slapdown.getPositionDegrees();
    // double target = ((Math.sin(time.get()*4) +
    // 1)/2)*(Constants.Slapdown.kSlapdownJiggleDownDegrees-Constants.Slapdown.kSlapdownJiggleUpDegrees);
    double center =
        (Constants.Slapdown.kSlapdownJiggleUpDegrees
                + Constants.Slapdown.kSlapdownJiggleDownDegrees)
            / 2.0;
    double amp =
        (Constants.Slapdown.kSlapdownJiggleDownDegrees
                - Constants.Slapdown.kSlapdownJiggleUpDegrees)
            / 2.0;
    double target = center + amp * Math.cos(time.get() * 2);

    // if (position >= Constants.Slapdown.kSlapdownJiggleDownDegrees) {
    //   this.slapdown.setPositionDegrees(Constants.Slapdown.kSlapdownJiggleUpDegrees);
    // } else if (position <= Constants.Slapdown.kSlapdownJiggleUpDegrees) {
    //   this.slapdown.setPositionDegrees(Constants.Slapdown.kSlapdownJiggleDownDegrees);
    // }
    this.slapdown.setPositionDegrees(target);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    this.slapdown.setPositionDegrees(Constants.Slapdown.kSlapdownDownPositionDegrees);
  }
}
