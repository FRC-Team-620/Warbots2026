package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.jmhsrobotics.frc2026.Constants;

public class SimIntakeIO implements IntakeIO {

  private static final DCMotor INTAKE_MOTOR = DCMotor.getNEO(1);
  private static final double INTAKE_GEAR_RATIO = 2.0; // Reduction
  public static final double IN_TO_KG_MOI = 0.0002926397;
  public static final double INTAKE_MOI = 0.4 * IN_TO_KG_MOI; // TODO: Use Real moI

  private final FlywheelSim intakeSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(INTAKE_MOTOR, INTAKE_MOI, INTAKE_GEAR_RATIO),
          INTAKE_MOTOR);

  private final PIDController intakePID = new PIDController(0.001, 0, 0);

  private boolean intakeOpenLoop = true;
  private double intakeVolts = 0.0;

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
  public void setIntakeSpeed(double dutyCycle) {
    intakeOpenLoop = false;
    intakePID.setSetpoint(dutyCycle * Constants.Intake.kBaseRPM); // Convert duty cycle to RPM
  }

  @Override
  public void setPositionDegrees(double degrees) {
    slapDownTargetRad = Math.toRadians(degrees);
  }

  @Override
  public void setPIDF(double p, double i, double d, double f) {
    intakePID.setPID(p, i, d);
  }

  @Override
  public void stop() {
    intakeOpenLoop = true;
    intakeVolts = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    /* -------- Intake Roller -------- */

    double intakeOutVolts;

    if (intakeOpenLoop) {
      intakeOutVolts = intakeVolts;
    } else {
      intakeOutVolts =
          MathUtil.clamp(intakePID.calculate(intakeSim.getAngularVelocityRPM()), -12.0, 12.0);
    }

    intakeSim.setInputVoltage(intakeOutVolts);
    intakeSim.update(Constants.ksimTimestep);

    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.RPM = intakeSim.getAngularVelocityRPM();
    inputs.intakeMotorTemperatureCelcius = 25.0;

    double slapDownVolts =
        MathUtil.clamp(
            slapDownPID.calculate(slapDownSim.getAngleRads(), slapDownTargetRad), -12.0, 12.0);

    slapDownSim.setInputVoltage(slapDownVolts);
    slapDownSim.update(Constants.ksimTimestep);

    inputs.slapDownPositionDegrees = Math.toDegrees(slapDownSim.getAngleRads());
    inputs.slapDownSpeedDegPerSec = Math.toDegrees(slapDownSim.getVelocityRadPerSec());
    inputs.slapDownCurrentAmps = slapDownSim.getCurrentDrawAmps();
    // inputs.slapDownAccelerationRPMPerSec =
    //     Units.radiansPerSecondToRotationsPerMinute(
    //             slapDownSim.getAccelerationRadPerSecSq())
    //         * 60.0;
  }

  @Override
  public void setIntakeBrakeMode(boolean enable) {
    // Sim brake mode is approximated by increased damping
    // TODO: implement
  }

  @Override
  public void setSlapDownBrakeMode(boolean enable) {
    // Sim brake mode is approximated by increased damping
    // TODO: implement
  }
}
