package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.CheckTolerance;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double setPointDegrees = Constants.Intake.kSlapDownUpPositionDegrees;

  private State calcluatedState = new State(Constants.Intake.kSlapDownUpPositionDegrees, 0);
  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(new Constraints(100, 200)); // TODO update these values

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    calcluatedState =
        trapezoidProfile.calculate(
            0.02,
            calcluatedState,
            new State(setPointDegrees, 0)); // 20ms is the default periodic rate
    intakeIO.setPositionDegrees(calcluatedState.position);

    intakeIO.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);
    Logger.recordOutput(
        "Model/Intake/arm_position",
        new Pose3d(
            0.252,
            0,
            0.204730,
            new Rotation3d(0, Units.degreesToRadians(inputs.slapDownPositionDegrees - 180), 0)));
    Logger.recordOutput(
        "Model/Intake/hopper_position",
        new Pose3d(
            MathUtil.clamp((inputs.slapDownPositionDegrees - 90) / 90, 0, 1) * 0.30188,
            0,
            0,
            new Rotation3d()));

    // /*
    // Logger.recordOutput("Intake/Intake Current Amps", inputs.intakeCurrentAmps);
    // Logger.recordOutput("Intake/Intake Speed RPM", inputs.RPM);
    // Logger.recordOutput("Intake/SlapDown Position Degrees", inputs.slapDownPositionDegrees);
    // Logger.recordOutput("Intake/SlapDown Current Amps", inputs.slapDownCurrentAmps);
    // Logger.recordOutput("Intake/Intake Temperature Celcius",
    // inputs.intakeMotorTemperatureCelcius);
    // */
  }

  public void set(double speedDutyCycle) {
    intakeIO.setSpeedDutyCycle(speedDutyCycle);
  }

  public boolean atGoal() {
    return CheckTolerance.atGoalTolerance(
        setPointDegrees,
        inputs.slapDownPositionDegrees,
        Constants.Intake.kSlapDownToleranceDegrees);
  }

  public void setPositionDegrees(double setPointDegrees) {
    this.setPointDegrees = setPointDegrees;
    intakeIO.setPositionDegrees(setPointDegrees);
  }

  public void setIntakeBrakeMode(boolean enable) {
    intakeIO.setIntakeBrakeMode(enable);
  }

  public void setSlapDownBrakeMode(boolean enable) {
    intakeIO.setSlapDownBrakeMode(enable);
  }

  public double getPositionDegrees() {
    return inputs.slapDownPositionDegrees;
  }
}
