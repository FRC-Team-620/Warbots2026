// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2026.Constants.OperatorConstants;
import org.jmhsrobotics.frc2026.commands.DriveTimeCommand;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.drive.GyroIOBoron;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIOThrifty;
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

  public final Drive drive;
  public final Shooter shooter;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Robot Mode
    if (Robot.isReal()) {
      drive =
          new Drive(
              new GyroIOBoron(),
              new ModuleIOThrifty(0),
              new ModuleIOThrifty(1),
              new ModuleIOThrifty(2),
              new ModuleIOThrifty(3));
      shooter = new Shooter(new NeoShooterIO());
      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

      // TODO: Tweak 'seconds' and 'velocityMPS' parameters of DriveTimeCommand to
      // updated values
      // (current values 2.2 and 0.3 are from 2025 season)
      autoChooser.addDefaultOption("BaseLineAuto", new DriveTimeCommand(2.2, 0.3, drive));
    }
    // Simulation Mode
    else {
      // TODO

      drive =
          new Drive(
              new GyroIOBoron(),
              new ModuleIOThrifty(0),
              new ModuleIOThrifty(1),
              new ModuleIOThrifty(2),
              new ModuleIOThrifty(3));
      shooter = new Shooter(new SimShooterIO());

      autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

      // TODO: Tweak 'seconds' and 'velocityMPS' parameters of DriveTimeCommand to
      // updated values
      // (current values 2.2 and 0.3 are from 2025 season)
      // autoChooser.addDefaultOption("BaseLineAuto", new DriveTimeCommand(2.2, 0.3, drive));
    }
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

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
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
