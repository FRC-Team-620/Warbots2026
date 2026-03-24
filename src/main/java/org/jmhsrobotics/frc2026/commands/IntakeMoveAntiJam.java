package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;

public class IntakeMoveAntiJam extends Command {
  private Intake intake;
  private double speedDutyCycle;

  Timer time = new Timer();
  boolean isUnjamming;
  double unjamSec = 0.5;
  double unjamPower = -1;
  double minIntakeTime = 0.5;

  public IntakeMoveAntiJam(Intake intake, double speedDutyCycle) {
    this.intake = intake;
    this.speedDutyCycle = speedDutyCycle;

    addRequirements(intake);

    // Should a min intake for min intake time. If the intake is stalled, start unjam for unjam
    // time. And have a minIntake time before unjam can trigger again
  }

  @Override
  public void initialize() {
    intake.set(speedDutyCycle);
    time.restart();
    isUnjamming = false;
  }

  @Override
  public void execute() {

    if (intake.isStalled() && !isUnjamming && time.hasElapsed(minIntakeTime)) {
      isUnjamming = true;
      time.restart();
    }

    if (isUnjamming && time.hasElapsed(unjamSec)) {
      isUnjamming = false;
      time.restart();
    }

    if (isUnjamming) {
      intake.set(unjamPower);
    } else {
      intake.set(speedDutyCycle);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intake.set(0);
    System.out.println("Intake Move (Anti Jam) Complete " + interrupted);
  }
}
