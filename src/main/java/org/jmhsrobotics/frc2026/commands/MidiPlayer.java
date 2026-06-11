package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.subsystems.climber.Climber;

public class Midiplayer extends Command {
  private ModuleIOTalonFX device;

  public ClimberRetractHooks(Climber climber) {
    Orchestra m_orchestra = new Orchestra();

    // Add a single device to the orchestra
    m_orchestra.addInstrument(m_motor);

    // Attempt to load the chrp
    var status = m_orchestra.loadMusic("");

    if (!status.isOK()) {
        // log error
    }
  }

  @Override
  public void initialize() {
    m_orchestra.play();
  }
}
