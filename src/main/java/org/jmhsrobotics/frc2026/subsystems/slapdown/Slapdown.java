package org.jmhsrobotics.frc2026.subsystems.slapdown;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.CheckTolerance;
import org.littletonrobotics.junction.Logger;

public class Slapdown extends SubsystemBase {
  private SlapdownIO slapdownIO;
  private SlapdownIOInputsAutoLogged inputs = new SlapdownIOInputsAutoLogged();
  private double setPointDegrees = Constants.Slapdown.kSlapdownUpPositionDegrees;
  private boolean isOpenLoop = false;

  private State calcluatedState = new State(Constants.Slapdown.kSlapdownUpPositionDegrees, 0);
  private TrapezoidProfile trapezoidProfile =
      new TrapezoidProfile(new Constraints(100, 200)); // TODO update these values

  private PIDController pidController =
      new PIDController(
          Constants.Slapdown.kSlapdownP,
          Constants.Slapdown.kSlapdownI,
          Constants.Slapdown.kSlapdownD);

  public Slapdown(SlapdownIO slapdownIO) {
    this.slapdownIO = slapdownIO;
    SmartDashboard.putNumber("Slapdown/pid/p", pidController.getP());
    SmartDashboard.putNumber("Slapdown/pid/i", pidController.getI());
    SmartDashboard.putNumber("Slapdown/pid/d", pidController.getD());
    // SmartDashboard.putData("Slapdown/pid", pidController);
  }

  @Override
  public void periodic() {
    if (!isOpenLoop) {
      calcluatedState =
          trapezoidProfile.calculate(
              0.02,
              calcluatedState,
              new State(setPointDegrees, 0)); // 20ms is the default periodic rate
      slapdownIO.setPositionDegrees(calcluatedState.position);
    }
    // slapdownIO.setPID(SmartDashboard.getNumber(, setPointDegrees), pidController.getI(),
    // pidController.getD());
    // slapdownIO.setPID(
    //     SmartDashboard.getNumber("Slapdown/pid/p", Constants.Slapdown.kSlapdownP),
    //     SmartDashboard.getNumber("Slapdown/pid/i", Constants.Slapdown.kSlapdownI),
    //     SmartDashboard.getNumber("Slapdown/pid/d", Constants.Slapdown.kSlapdownD));
    calcluatedState =
        trapezoidProfile.calculate(
            0.02,
            calcluatedState,
            new State(setPointDegrees, 0)); // 20ms is the default periodic rate
    slapdownIO.setPositionDegrees(calcluatedState.position);

    slapdownIO.updateInputs(inputs);

    Logger.processInputs("Slapdown", inputs);
    Logger.recordOutput(
        "Model/Slapdown/arm_position_goal",
        new Pose3d(
            0.252,
            0,
            0.204730,
            new Rotation3d(0, Units.degreesToRadians(setPointDegrees - 180), 0)));
    Logger.recordOutput(
        "Model/Slapdown/hopper_position_goal",
        new Pose3d(
            MathUtil.clamp((setPointDegrees - 90) / 90, 0, 1) * 0.30188, 0, 0, new Rotation3d()));
    Logger.recordOutput(
        "Model/Slapdown/arm_position",
        new Pose3d(
            0.252,
            0,
            0.204730,
            new Rotation3d(0, Units.degreesToRadians(inputs.slapdownPositionDegrees - 180), 0)));
    Logger.recordOutput(
        "Model/Slapdown/hopper_position",
        new Pose3d(
            MathUtil.clamp((inputs.slapdownPositionDegrees - 90) / 90, 0, 1) * 0.30188,
            0,
            0,
            new Rotation3d()));
  }

  public boolean atGoal() {
    return CheckTolerance.atGoalTolerance(
        setPointDegrees,
        inputs.slapdownPositionDegrees,
        Constants.Slapdown.kSlapdownToleranceDegrees);
  }

  public void setPositionDegrees(double setPointDegrees) {
    this.setPointDegrees = setPointDegrees;
    this.isOpenLoop = false;
    slapdownIO.setPositionDegrees(setPointDegrees);
  }

  public void setSlapdownBrakeMode(boolean enable) {
    slapdownIO.setSlapdownBrakeMode(enable);
  }

  public void setSpeedDutyCycle(double dutyCycle) {
    this.isOpenLoop = true;
    slapdownIO.setSpeedDutyCycle(dutyCycle);
  }

  public double getPositionDegrees() {
    return inputs.slapdownPositionDegrees;
  }

  public boolean canIntake() {
    return this.atGoal() && this.setPointDegrees == Constants.Slapdown.kSlapdownDownPositionDegrees;
  }

  public double getCurrentAmps() {
    return inputs.slapdownCurrentAmps;
  }

  public double getAbsPositionDegrees() {
    return inputs.slapdownAbsPositionDegrees;
  }

  public void setSlapdownEncoder(double positionDegrees) {
    slapdownIO.setSlapdownEncoder(positionDegrees);
  }
}
