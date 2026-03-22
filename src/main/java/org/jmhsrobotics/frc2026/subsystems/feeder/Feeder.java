package org.jmhsrobotics.frc2026.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  public FeederIO feederIO;
  private FeederIOInputsAutoLogged feederInputs = new FeederIOInputsAutoLogged();
  private double GoalDutyCycle;

  public Feeder(FeederIO feederIO) {
    this.feederIO = feederIO;
    GoalDutyCycle = 0;
  }

  @Override
  public void periodic() {
    feederIO.updateInputs(feederInputs);

    Logger.processInputs("/Feeder", feederInputs);
    Logger.recordOutput("Feeder/GoalDutyCycle", GoalDutyCycle);
  }

  public void setFeederSpeed(double dutyCycle) {
    this.GoalDutyCycle = dutyCycle;
    feederIO.setFeederSpeed(dutyCycle);
  }
}
