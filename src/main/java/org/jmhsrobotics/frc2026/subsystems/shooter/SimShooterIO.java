package org.jmhsrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jmhsrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class SimShooterIO implements ShooterIO {
  public static final double IN_TO_KG_MOI = 0.0002926397; // TODO: Move to constants
  public static final double MOI = 19.479 * IN_TO_KG_MOI; // TODO add units to var name
  public static final DCMotor MOTOR = DCMotor.getNEO(3);
  public static final double GEEARING = 1;

  FlywheelSim flywheelSim =
      new FlywheelSim(LinearSystemId.createFlywheelSystem(MOTOR, MOI, GEEARING), MOTOR);

  PIDController pid;
  public boolean isOpenLoop = false;
  public double outputVolts = 0;
  public double goalRPM = 0;

  SimpleMotorFeedforward feedForward;

  public SimShooterIO(
      double k, double i, double d) { // TODO: This should prob be stored elsewhere (pid gains)
    pid = new PIDController(k, i, d);
    feedForward = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV);

    SmartDashboard.putData("sim/flywheel/pid", pid);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double outvolts = this.outputVolts;
    if (!isOpenLoop) {
      outvolts =
          MathUtil.clamp(
              this.pid.calculate(flywheelSim.getAngularVelocityRPM())
                  + feedForward.calculate(this.goalRPM),
              -RobotController.getBatteryVoltage(),
              RobotController.getBatteryVoltage());
    }
    this.flywheelSim.setInputVoltage(outvolts);
    this.flywheelSim.update(Constants.ksimTimestep);

    inputs.currentAMPS = flywheelSim.getCurrentDrawAmps();
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.voltage = flywheelSim.getInputVoltage();
    inputs.tempC = 20;
    inputs.positionROT += (flywheelSim.getAngularVelocityRPM() / 60.0) * Constants.ksimTimestep;
    inputs.goalRPM = this.goalRPM;
    Logger.recordOutput("Shooter/VelocityRPS", inputs.velocityRPM / 60.0);
  }

  @Override
  public void setSpeed(double speed) {
    isOpenLoop = true;
    outputVolts = MathUtil.clamp(speed * RobotController.getBatteryVoltage(), -13, 13);
  }

  @Override
  public void setRPM(double RPM) {
    this.goalRPM = RPM;
    isOpenLoop = false;
    pid.setSetpoint(RPM);
  }

  public void setVoltage(double voltage) {
    this.outputVolts = voltage;
    isOpenLoop = true;
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // TODO: Implement in SIm
  }

  @Override
  public void setPIDF(double p, double i, double d, double f) {
    pid.setPID(p, i, d);
    // TODO: Implement Feed forward

  }

  @Override
  public void stop() {
    // Set motor to open loop mode and set output voltage to 0
    this.isOpenLoop = true;
    this.outputVolts = 0;
  }
}
