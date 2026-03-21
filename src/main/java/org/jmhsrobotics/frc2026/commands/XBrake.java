package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;

public class XBrake extends Command {
  private Drive drive;

  private Timer timer = new Timer();

  public XBrake(Drive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.stopWithX();
  }

  @Override
  public void execute() {
    drive.stopWithX();
  }
}
