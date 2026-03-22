package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;

public class TuneRPMCommand extends Command {
  Shooter shooter;

  public TuneRPMCommand(Shooter shooter) {
    this.shooter = shooter;
    SmartDashboard.putNumber("Tune-RPM", 3000);
  }

  @Override
  public void execute() {
    var rpm = SmartDashboard.getNumber("Tune-RPM", 0);
    if (rpm == 0) {
      this.shooter.stop();
    } else {
      this.shooter.setRPM(SmartDashboard.getNumber("Tune-RPM", 0));
    }
  }
}
