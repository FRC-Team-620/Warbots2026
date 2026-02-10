package org.jmhsrobotics.frc2026.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO indexerIO;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO) {
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(inputs);

    Logger.processInputs("/indexer", inputs);

    Logger.recordOutput("Indexer/Motor RPM", inputs.motorRPM);
    Logger.recordOutput("Indexer/Motor Current Amps", inputs.currentAMPS);
    Logger.recordOutput("Indexer/Motor Temperature Celcius", inputs.motorTemperatureCelcius);
  }

  public void set(double speedDutyCycle) {
    indexerIO.set(speedDutyCycle);
  }

  public void setBrakeMode(boolean enable) {
    indexerIO.setBrakeMode(enable);
  }
}
