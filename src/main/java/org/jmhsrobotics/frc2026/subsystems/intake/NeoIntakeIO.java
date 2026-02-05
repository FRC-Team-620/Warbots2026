package org.jmhsrobotics.frc2026.subsystems.intake;

import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.util.SparkUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class NeoIntakeIO {
    private SparkMax intakeMotor = new SparkMax(Constants.CAN.kIntakeMotorID, MotorType.kBrushless);
    private SparkMaxConfig intakeMotorConfig;
    private SparkMax slapDownMotor = new SparkMax(Constants.CAN.kSlapDownMotorID, MotorType.kBrushless);
    private SparkMaxConfig slapDownMotorConfig;
    private RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private RelativeEncoder slapDownEncoder = slapDownMotor.getEncoder();
    private SparkClosedLoopController intakePIDController;
    private SparkClosedLoopController slapDownPIDController;
    private double speedRPM;
    private double angleDegrees;


    public NeoIntakeIO(){
        intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20)
            .voltageCompensation(12)
            .inverted(false)
            .closedLoop
            .pid(Constants.Intake.kP, Constants.Intake.kI, Constants.Intake.kD)
            .maxOutput(1)
            .minOutput(0)
            .maxMotion
            .cruiseVelocity(980)
            .maxAcceleration(20000);

        SparkUtil.tryUntilOk(intakeMotor, 5, () ->
            intakeMotor.configure(
                intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
        }

        intakePIDController = intakeMotor.getClosedLoopController();

}
