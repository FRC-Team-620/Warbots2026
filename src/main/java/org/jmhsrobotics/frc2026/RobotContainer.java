// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2026;

import com.pathplanner.lib.auto.AutoBuilder;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.jmhsrobotics.frc2026.commands.ClimberExtendHooks;
import org.jmhsrobotics.frc2026.commands.ClimberMove;
import org.jmhsrobotics.frc2026.commands.ClimberRetractHooks;
import org.jmhsrobotics.frc2026.commands.DriveCommand;
import org.jmhsrobotics.frc2026.commands.DriveTimeCommand;
import org.jmhsrobotics.frc2026.commands.IndexerMove;
import org.jmhsrobotics.frc2026.commands.IntakeMove;
import org.jmhsrobotics.frc2026.commands.LEDToControlMode;
import org.jmhsrobotics.frc2026.commands.ShooterMove;
import org.jmhsrobotics.frc2026.commands.SlapdownMove;
import org.jmhsrobotics.frc2026.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2026.controlBoard.DoubleControl;
import org.jmhsrobotics.frc2026.subsystems.climber.Climber;
import org.jmhsrobotics.frc2026.subsystems.climber.ClimberIO;
import org.jmhsrobotics.frc2026.subsystems.climber.NeoClimberIO;
import org.jmhsrobotics.frc2026.subsystems.climber.SimClimberIO;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.drive.GyroIO;
import org.jmhsrobotics.frc2026.subsystems.drive.GyroIOBoron;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIO;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIOSimRev;
import org.jmhsrobotics.frc2026.subsystems.drive.swerve.ModuleIOThrifty;
import org.jmhsrobotics.frc2026.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2026.subsystems.indexer.IndexerIO;
import org.jmhsrobotics.frc2026.subsystems.indexer.NeoIndexerIO;
import org.jmhsrobotics.frc2026.subsystems.indexer.SimIndexerIO;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;
import org.jmhsrobotics.frc2026.subsystems.intake.IntakeIO;
import org.jmhsrobotics.frc2026.subsystems.intake.NeoIntakeIO;
import org.jmhsrobotics.frc2026.subsystems.intake.SimIntakeIO;
import org.jmhsrobotics.frc2026.subsystems.led.LED;
import org.jmhsrobotics.frc2026.subsystems.shooter.NeoShooterIO;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;
import org.jmhsrobotics.frc2026.subsystems.shooter.ShooterIO;
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
  private final Indexer indexer;
  private final Climber climber;

  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putString("/CurrentSimMode", Constants.currentMode.toString());
    SmartDashboard.putData(CommandScheduler.getInstance());

    CanandEventLoop.getInstance();

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

        shooter = new Shooter(new NeoShooterIO());
        intake = new Intake(new NeoIntakeIO());
        indexer = new Indexer(new NeoIndexerIO());
        climber = new Climber(new NeoClimberIO());
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

        shooter =
            new Shooter(
                new SimShooterIO(
                    Constants.ShooterConstants.kP,
                    Constants.ShooterConstants.kI,
                    Constants.ShooterConstants.kD) {});

        // FIXME:add SimIntakeIO
        intake = new Intake(new SimIntakeIO());
        indexer = new Indexer(new SimIndexerIO());
        climber = new Climber(new SimClimberIO());
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

        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        climber = new Climber(new ClimberIO() {});
        break;
    }

    this.control = new DoubleControl();

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
    // default commands
    drive.setDefaultCommand(new DriveCommand(drive, control));
    // intake.setDefaultCommand(new SlapdownMove(intake, 90));
    indexer.setDefaultCommand(new IndexerMove(indexer, 0.0));
    // climber.setDefaultCommand(new ClimberMove(climber, 0)); // TODO figure out real parameters
    // for climber move

    // Shooter Bindings
    control.shoot().onTrue(new ShooterMove(shooter, Constants.ShooterConstants.kBaseRPM));
    control.shoot().onFalse(new ShooterMove(shooter, 0));

    // Slapdown Bindings
    control.SlapdownMoveDown()
        .onTrue(new SlapdownMove(intake, Constants.Intake.kSlapDownDownPositionDegrees));
    control.SlapdownMoveUp()
        .onTrue(new SlapdownMove(intake, Constants.Intake.kSlapDownUpPositionDegrees));

    // Intake Bindings
    control.intakeOn().onTrue(new IntakeMove(intake, Constants.Intake.kSpeedDutyCycle));
    control.intakeOff().onTrue(new IntakeMove(intake, 0));
    control.extakeFuel().onTrue(new IntakeMove(intake, -(Constants.Intake.kSpeedDutyCycle)));

    // Indexer Binding
    control.indexOn().onTrue(new IndexerMove(indexer, Constants.Indexer.kSpeedDutyCycle));
    control.indexOn().onFalse(new IndexerMove(indexer, 0));

    // Climber Bindings
    control
        .climberUp()
        .onTrue(new ClimberMove(climber, Constants.Climber.kSpeedDutyCycle))
        .onFalse(new ClimberMove(climber, 0));
    control
        .climberDown()
        .onTrue(new ClimberMove(climber, -Constants.Climber.kSpeedDutyCycle))
        .onFalse(new ClimberMove(climber, 0));

    control.turbo().onTrue(Commands.runOnce(() -> drive.setTurboMode(true)));
    control.turbo().onFalse(Commands.runOnce(() -> drive.setTurboMode(false)));

    control
        .resetForward()
        .onTrue(
            Commands.runOnce(
                () -> {
                  boolean isRed =
                      DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get() == Alliance.Red;
                  drive.setPose(
                      new Pose2d(
                          drive.getPose().getTranslation(),
                          Rotation2d.fromDegrees(isRed ? 180 : 0)));
                },
                drive));

    SmartDashboard.putData("Indexer Full Speed", new IndexerMove(indexer, 1));
    SmartDashboard.putData("Indexer Stop", new IndexerMove(indexer, 0));
    SmartDashboard.putData("Indexer Half Speed", new IndexerMove(indexer, 0.5));
    SmartDashboard.putData("Climber Up", new ClimberMove(climber, 30));
    SmartDashboard.putData("Climber Down", new ClimberMove(climber, 0));
    SmartDashboard.putData("Climber Extend", new ClimberExtendHooks(climber));
    SmartDashboard.putData("Climber Retract", new ClimberRetractHooks(climber));
    // SmartDashboard.putData("Intake Full Speed", new IntakeMove(intake));
    SmartDashboard.putData(
        "Shooter Run", new ShooterMove(shooter, Constants.ShooterConstants.kBaseRPM));
    SmartDashboard.putData("Intake Move", new IntakeMove(intake, Constants.Intake.kSpeedDutyCycle));
    SmartDashboard.putData("Shooter Stop", new ShooterMove(shooter, 0));
    SmartDashboard.putData("Slapdown Down", new SlapdownMove(intake, 180));
    SmartDashboard.putData("Slapdown Up", new SlapdownMove(intake, 90.0));
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
