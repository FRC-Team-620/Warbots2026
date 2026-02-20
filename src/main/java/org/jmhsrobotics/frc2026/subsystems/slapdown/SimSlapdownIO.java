package org.jmhsrobotics.frc2026.subsystems.slapdown;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.jmhsrobotics.frc2026.Constants;

public class SimSlapdownIO implements SlapdownIO {
  public static final double IN_TO_KG_MOI = 0.0002926397;

  private static final DCMotor SLAPDOWN_MOTOR = DCMotor.getNEO(1);
  private static final double SLAPDOWN_GEAR_RATIO = 40.0;
  private static final double SLAPDOWN_ARM_LENGTH_METERS = 0.4;
  private static final double SLAPDOWN_MIN_ANGLE_RAD = Math.toRadians(0);
  private static final double SLAPDOWN_MAX_ANGLE_RAD = Math.toRadians(180);

  private static final double SLAPDOWN_MOI = 0.4 * IN_TO_KG_MOI; // TODO: Real MOI

  private final SingleJointedArmSim slapDownSim =
      new SingleJointedArmSim(
          SLAPDOWN_MOTOR,
          SLAPDOWN_GEAR_RATIO,
          SLAPDOWN_MOI,
          SLAPDOWN_ARM_LENGTH_METERS,
          SLAPDOWN_MIN_ANGLE_RAD,
          SLAPDOWN_MAX_ANGLE_RAD,
          true,
          0.0);

  private final PIDController slapDownPID = new PIDController(4.0, 0.0, 0.2);
  private double slapDownTargetRad = 0.0;

  @Override
  public void setPositionDegrees(double degrees) {
    slapDownTargetRad = Math.toRadians(degrees);
  }

  @Override
  public void updateInputs(SlapdownIOInputs inputs) {
    double slapDownVolts =
        MathUtil.clamp(
            slapDownPID.calculate(slapDownSim.getAngleRads(), slapDownTargetRad), -12.0, 12.0);

    slapDownSim.setInputVoltage(slapDownVolts);
    slapDownSim.update(Constants.ksimTimestep);

    inputs.slapdownPositionDegrees = Math.toDegrees(slapDownSim.getAngleRads());
    inputs.slapdownSpeedDegPerSec = Math.toDegrees(slapDownSim.getVelocityRadPerSec());
    inputs.slapdownCurrentAmps = slapDownSim.getCurrentDrawAmps();
    // inputs.slapDownAccelerationRPMPerSec =
    //     Units.radiansPerSecondToRotationsPerMinute(
    //             slapDownSim.getAccelerationRadPerSecSq())
    //         * 60.0;
  }

  @Override
  public void setSlapdownBrakeMode(boolean enable) {
    // Sim brake mode is approximated by increased damping
    // TODO: implement
  }
}
