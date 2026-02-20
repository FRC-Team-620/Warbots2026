package org.jmhsrobotics.frc2026.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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

  @Override
  public void setSpeedDutyCycle(double dutyCycle) {
    intakeOpenLoop = false;
    intakePID.setSetpoint(dutyCycle * Constants.Intake.kBaseRPM); // Convert duty cycle to RPM
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
  }

  @Override
  public void setIntakeBrakeMode(boolean enable) {
    // Sim brake mode is approximated by increased damping
    // TODO: implement
  }
}
