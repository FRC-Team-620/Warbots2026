package org.jmhsrobotics.frc2026.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  public FeederIO feederIO;
  private FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO feederIO) {
    this.feederIO = feederIO;
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);

    Logger.processInputs("/Feeder", feederInputs);
  }

  public void setFeederSpeed(double dutyCycle) {
    feederIO.setFeederSpeed(dutyCycle);
  }
}
