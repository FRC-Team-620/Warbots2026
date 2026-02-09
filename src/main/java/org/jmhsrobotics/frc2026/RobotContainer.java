// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2026.commands.DriveCommands;
import org.jmhsrobotics.frc2026.commands.DriveTimeCommand;
import org.jmhsrobotics.frc2026.commands.IntakeMove;
import org.jmhsrobotics.frc2026.commands.LEDToControlMode;
import org.jmhsrobotics.frc2026.commands.ShooterMove;
import org.jmhsrobotics.frc2026.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2026.controlBoard.SingleControl;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.drive.GyroIO;
import org.jmhsrobotics.frc2026.subsystems.drive.GyroIOBoron;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIO;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIOSimRev;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIOThrifty;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;
import org.jmhsrobotics.frc2026.subsystems.intake.IntakeIO;
import org.jmhsrobotics.frc2026.subsystems.intake.NeoIntakeIO;
import org.jmhsrobotics.frc2026.subsystems.led.LED;
import org.jmhsrobotics.frc2026.subsystems.shooter.NeoShooterIO;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;
import org.jmhsrobotics.frc2026.subsystems.shooter.SimShooterIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Shooter shooter;
  private final LED led;
  private final ControlBoard control;
  private final Intake intake;

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putString("/CurrentSimMode", Constants.currentMode.toString());
    SmartDashboard.putData(CommandScheduler.getInstance());
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOThrifty(0),
                new ModuleIOThrifty(1),
                new ModuleIOThrifty(2),
                new ModuleIOThrifty(3));

        shooter = new Shooter(new NeoShooterIO() {});
        intake = new Intake(new NeoIntakeIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev(),
                new ModuleIOSimRev());

        // FIXME:add SimShooterIO
        shooter =
            new Shooter(
                new SimShooterIO(
                    Constants.ShooterConstants.kP,
                    Constants.ShooterConstants.kI,
                    Constants.ShooterConstants.kD) {});
        // FIXME:add SimIntakeIO
        intake = new Intake(new NeoIntakeIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shooter = new Shooter(new NeoShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        break;
    }

    this.control = new SingleControl();

    led = new LED();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // TODO: Tweak 'seconds' and 'velocityMPS' parameters of DriveTimeCommand to updated values
    // (current values 2.2 and 0.3 are from 2025 season)
    autoChooser.addDefaultOption("BaseLineAuto", new DriveTimeCommand(2.2, 0.3, drive));

    // Configure the trigger bindings
    configureBindings();
    configureDriverFeedback();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    drive.setDefaultCommand(
        DriveCommands.joystickDriveAtAngle(
            drive, control::translationX, control::translationY, control::rotationABS));
    shooter.setDefaultCommand(new ShooterMove(shooter, control.shoot()));
    intake.setDefaultCommand(new IntakeMove(intake));
  }

  /**
   * Use this method to change LED's on Robot based on things happening during the match. I know,
   * I'm great at documentation :)
   */
  // TODO: Actually test this to make sure it works correctly
  private void configureDriverFeedback() {
    led.setDefaultCommand(new LEDToControlMode(this.led));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
