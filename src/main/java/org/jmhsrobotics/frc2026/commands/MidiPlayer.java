package org.jmhsrobotics.frc2026.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class MidiPlayer extends Command {

  private Orchestra sing;

  public MidiPlayer(TalonFX device, TalonFX device2, String filepath) {

    this.sing = new Orchestra();

    sing.addInstrument(device);
    sing.addInstrument(device2);
    sing.loadMusic(filepath);
  }

  @Override
  public void initialize() {
    sing.play();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    sing.stop();
    sing.clearInstruments();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
