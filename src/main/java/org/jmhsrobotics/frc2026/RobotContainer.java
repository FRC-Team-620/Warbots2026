// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2026;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.jmhsrobotics.frc2026.commands.AimingAuto;
import org.jmhsrobotics.frc2026.commands.AlignToHub;
import org.jmhsrobotics.frc2026.commands.ClimberExtendHooks;
import org.jmhsrobotics.frc2026.commands.ClimberMove;
import org.jmhsrobotics.frc2026.commands.ClimberRetractHooks;
import org.jmhsrobotics.frc2026.commands.DistanceAdjustingShoot;
import org.jmhsrobotics.frc2026.commands.DriveCommand;
import org.jmhsrobotics.frc2026.commands.DriveTimeCommand;
import org.jmhsrobotics.frc2026.commands.FaceDriveDirection;
import org.jmhsrobotics.frc2026.commands.Feed;
import org.jmhsrobotics.frc2026.commands.HoodDown;
import org.jmhsrobotics.frc2026.commands.IndexerMove;
import org.jmhsrobotics.frc2026.commands.IntakeMove;
import org.jmhsrobotics.frc2026.commands.IntakeMoveAntiJam;
import org.jmhsrobotics.frc2026.commands.LEDToControlMode;
import org.jmhsrobotics.frc2026.commands.PreloadAuto;
import org.jmhsrobotics.frc2026.commands.SetSlapdownToAbs;
import org.jmhsrobotics.frc2026.commands.ShooterSetDutyCycle;
import org.jmhsrobotics.frc2026.commands.ShooterSpinup;
import org.jmhsrobotics.frc2026.commands.SlapdownJiggle;
import org.jmhsrobotics.frc2026.commands.SlapdownMove;
import org.jmhsrobotics.frc2026.commands.TuneRPMCommand;
import org.jmhsrobotics.frc2026.commands.ZeroSlapdownCommand;
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
import org.jmhsrobotics.frc2026.subsystems.feeder.Feeder;
import org.jmhsrobotics.frc2026.subsystems.feeder.FeederIO;
import org.jmhsrobotics.frc2026.subsystems.feeder.NeoFeederIO;
import org.jmhsrobotics.frc2026.subsystems.feeder.SimFeederIO;
import org.jmhsrobotics.frc2026.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2026.subsystems.indexer.IndexerIO;
import org.jmhsrobotics.frc2026.subsystems.indexer.NeoIndexerIO;
import org.jmhsrobotics.frc2026.subsystems.indexer.SimIndexerIO;
import org.jmhsrobotics.frc2026.subsystems.intake.Intake;
import org.jmhsrobotics.frc2026.subsystems.intake.IntakeIO;
import org.jmhsrobotics.frc2026.subsystems.intake.SimIntakeIO;
import org.jmhsrobotics.frc2026.subsystems.intake.VortexIntakeIO;
import org.jmhsrobotics.frc2026.subsystems.led.LED;
import org.jmhsrobotics.frc2026.subsystems.shooter.NeoShooterIO;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;
import org.jmhsrobotics.frc2026.subsystems.shooter.ShooterIO;
import org.jmhsrobotics.frc2026.subsystems.shooter.SimShooterIO;
import org.jmhsrobotics.frc2026.subsystems.slapdown.NeoSlapdownIO;
import org.jmhsrobotics.frc2026.subsystems.slapdown.SimSlapdownIO;
import org.jmhsrobotics.frc2026.subsystems.slapdown.Slapdown;
import org.jmhsrobotics.frc2026.subsystems.slapdown.SlapdownIO;
import org.jmhsrobotics.frc2026.subsystems.vision.Vision;
import org.jmhsrobotics.frc2026.subsystems.vision.VisionConstants;
import org.jmhsrobotics.frc2026.subsystems.vision.VisionIO;
import org.jmhsrobotics.frc2026.subsystems.vision.VisionIOPhotonVision;
import org.jmhsrobotics.frc2026.subsystems.vision.VisionIOPhotonVisionSim;
import org.jmhsrobotics.frc2026.util.BallTracker;
import org.jmhsrobotics.frc2026.util.FuelSim;
import org.littletonrobotics.junction.Logger;
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
  public final Slapdown slapdown;
  private final Indexer indexer;
  private final Climber climber;
  private final Vision vision;
  private final Feeder feeder;
  private final SysIdRoutine routine;

  private final LoggedDashboardChooser<Command> autoChooser;

  public FuelSim fuelSim = new FuelSim("FuelSim");
  public BallTracker ballTracker;

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
        // Old Code - Keep here in case we need to revert to Neo Motor
        // intake = new Intake(new NeoIntakeIO());
        intake = new Intake(new VortexIntakeIO());
        slapdown = new Slapdown(new NeoSlapdownIO());
        indexer = new Indexer(new NeoIndexerIO());
        climber = new Climber(new NeoClimberIO());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        feeder = new Feeder(new NeoFeederIO());
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
                    0.06, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD) {});

        intake = new Intake(new SimIntakeIO());
        indexer = new Indexer(new SimIndexerIO());
        slapdown = new Slapdown(new SimSlapdownIO());
        climber = new Climber(new SimClimberIO());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        feeder = new Feeder(new SimFeederIO());
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
        slapdown = new Slapdown(new SlapdownIO() {});
        indexer = new Indexer(new IndexerIO() {});
        climber = new Climber(new ClimberIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        feeder = new Feeder(new FeederIO() {});
        break;
    }

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(shooter::setVoltage, (null), shooter));
    this.control = new DoubleControl();

    led = new LED();

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // TODO: Tweak 'seconds' and 'velocityMPS' parameters of DriveTimeCommand to updated values
    // (current values 2.2 and 0.3 are from 2025 season)
    autoChooser.addDefaultOption("BaseLineAuto", new DriveTimeCommand(2.2, 0.3, drive));
    autoChooser.addOption(
        "FrontHubAutoBLUE",
        new PreloadAuto(drive, shooter, indexer, feeder, Constants.Auto.hubStartBLUE));
    autoChooser.addOption(
        "LeftTrenchAutoBLUE",
        new AimingAuto(
            drive, shooter, indexer, feeder, Constants.Auto.leftTrenchStartBLUE, control));
    autoChooser.addOption(
        "LeftBumpAutoBLUE",
        new AimingAuto(drive, shooter, indexer, feeder, Constants.Auto.leftBumpStartBLUE, control));
    autoChooser.addOption(
        "RightTrenchAutoBLUE",
        new AimingAuto(
            drive, shooter, indexer, feeder, Constants.Auto.rightTrenchStartBLUE, control));
    autoChooser.addOption(
        "RightBumpAutoBLUE",
        new AimingAuto(
            drive, shooter, indexer, feeder, Constants.Auto.rightBumpStartBLUE, control));

    autoChooser.addOption(
        "FrontHubAutoRED",
        new PreloadAuto(drive, shooter, indexer, feeder, Constants.Auto.hubStartRED));
    autoChooser.addOption(
        "LeftTrenchAutoRED",
        new AimingAuto(
            drive, shooter, indexer, feeder, Constants.Auto.leftTrenchStartRED, control));
    autoChooser.addOption(
        "LeftBumpAutoRED",
        new AimingAuto(drive, shooter, indexer, feeder, Constants.Auto.leftBumpStartRED, control));
    autoChooser.addOption(
        "RightTrenchAutoRED",
        new AimingAuto(
            drive, shooter, indexer, feeder, Constants.Auto.rightTrenchStartRED, control));
    autoChooser.addOption(
        "RightBumpAutoRED",
        new AimingAuto(drive, shooter, indexer, feeder, Constants.Auto.rightBumpStartRED, control));
    // Configure the trigger bindings

    ballTracker = new BallTracker(drive::getPose, 10, 3);
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
    // shooter.setDefaultCommand(new ShooterSpinup(shooter, 1300));
    // led.setDefaultCommand(getAutonomousCommand());

    // Shooter Bindings
    control
        .shooterSpinup()
        .onTrue(new DistanceAdjustingShoot(shooter, drive))
        .onFalse(new ShooterSpinup(shooter, 0));

    // control
    //     .shooterSpinup()
    //     .onTrue(new ShooterSetDutyCycle(shooter, Constants.ShooterConstants.kShooterDutyCycle))
    //     .onFalse(new ShooterSetDutyCycle(shooter, 0));

    control
        .dutyCycleShoot()
        .whileTrue(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    Commands.runOnce(() -> shooter.setHoodPosition(0.31)),
                    new ShooterSpinup(shooter, Constants.ShooterConstants.kHubSetPointRPM))));
    // new SlapdownJiggle(slapdown),
    // new IntakeMoveAntiJam(intake, Constants.Intake.kSpeedDutyCycle)));
    // new IntakeMove(intake, Constants.Intake.kSpeedDutyCycle)));

    control
        .feedAndShoot()
        .onTrue(
            new ParallelCommandGroup(
                new Feed(feeder, Constants.Feeder.kSpeedDutyCycle, shooter),
                new DistanceAdjustingShoot(shooter, drive)))
        .onFalse(
            new ParallelCommandGroup(
                new Feed(feeder, 0, shooter), new ShooterSetDutyCycle(shooter, 0)));

    control
        .runFeeder()
        .whileTrue(
            new ParallelCommandGroup(
                new Feed(feeder, Constants.Feeder.kSpeedDutyCycle, shooter),
                // new IndependentFeed(feeder, Constants.Feeder.kSpeedDutyCycle),
                // new IntakeMoveAntiJam(intake, Constants.Intake.kSpeedDutyCycle),
                new WaitCommand(0.6).andThen(new SlapdownJiggle(slapdown))));
    // .onFalse(new IndependentFeed(feeder, 0));

    control.hoodDown().onTrue(new HoodDown(shooter));

    // Slapdown Bindings
    control
        .slapdownMoveDown()
        .onTrue(
            new SequentialCommandGroup(
                new SlapdownMove(slapdown, Constants.Slapdown.kSlapdownDownPositionDegrees)
                    .withTimeout(1.5),
                new IntakeMoveAntiJam(intake, Constants.Intake.kSpeedDutyCycle)));
    control
        .slapdownMoveUp()
        .onTrue(
            new ParallelRaceGroup(
                new IntakeMove(intake, Constants.Intake.kSpeedDutyCycle / 3),
                new SlapdownMove(slapdown, Constants.Slapdown.kSlapdownUpPositionDegrees)));

    // Intake Bindings
    control
        .intakeOn()
        .onTrue(
            new ParallelCommandGroup(
                new IntakeMoveAntiJam(intake, Constants.Intake.kSpeedDutyCycle),
                Commands.run(
                        () ->
                            led.setPattern(LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.1))))
                    .withTimeout(1.5)));
    control.intakeOff().onTrue(new IntakeMove(intake, 0));
    control.extakeFuel().onTrue(new IntakeMoveAntiJam(intake, -(Constants.Intake.kSpeedDutyCycle)));

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
    control.slowdown().onTrue(Commands.runOnce(() -> drive.setSlowdownMode(true)));
    control.slowdown().onFalse(Commands.runOnce(() -> drive.setSlowdownMode(false)));

    // extend climber
    control.ClimberExtendHooks().onTrue(new ClimberExtendHooks(climber));
    // retract climber
    control.ClimberRetractHooks().onTrue(new ClimberRetractHooks(climber));

    control.autoAim().whileTrue(new AlignToHub(drive, control));
    control.faceDriveDirection().whileTrue(new FaceDriveDirection(drive, control));

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
    SmartDashboard.putData(
        "Climber Up", new ClimberMove(climber, Constants.Climber.kSpeedDutyCycle));
    SmartDashboard.putData(
        "Climber Down", new ClimberMove(climber, -Constants.Climber.kSpeedDutyCycle));
    SmartDashboard.putData("Climber Stop", new ClimberMove(climber, 0));
    SmartDashboard.putData("Climber Extend", new ClimberExtendHooks(climber));
    SmartDashboard.putData("Climber Retract", new ClimberRetractHooks(climber));
    // SmartDashboard.putData("Intake Full Speed", new IntakeMove(intake));
    SmartDashboard.putData(
        "Shooter Spinup", new ShooterSpinup(shooter, Constants.ShooterConstants.kBaseRPM));
    SmartDashboard.putData("Shooter Stop", new ShooterSpinup(shooter, 0));
    SmartDashboard.putData("Feed", new Feed(feeder, Constants.Feeder.kSpeedDutyCycle, shooter));
    SmartDashboard.putData("Intake Move", new IntakeMove(intake, Constants.Intake.kSpeedDutyCycle));
    SmartDashboard.putData(
        "Slapdown Down", new SlapdownMove(slapdown, 180)); // TODO: Add to Constants
    SmartDashboard.putData("Slapdown Up", new SlapdownMove(slapdown, 65.0));
    SmartDashboard.putData("Slapdown Jiggle", new SlapdownJiggle(slapdown));
    SmartDashboard.putData("AutoAlignHub", new AlignToHub(drive, control));
    SmartDashboard.putData("Face Drive Direction", new FaceDriveDirection(drive, control));
    SmartDashboard.putData("Shooter Duty Cycle", new ShooterSetDutyCycle(shooter, 0.5));

    SmartDashboard.putData("DistanceAdjustingShoot", new DistanceAdjustingShoot(shooter, drive));

    SmartDashboard.putData("TuneFlywheel", new TuneRPMCommand(shooter));

    SmartDashboard.putData("Zero Slapdown", new ZeroSlapdownCommand(slapdown, 0.3, 20, 60, -0.1));
    SmartDashboard.putData("Set Slapdown to Absolute", new SetSlapdownToAbs(slapdown));
    // SmartDashboard.putData("autoCmds/frontHubAuto", new PreloadAuto(drive, shooter,
    // Constants.Auto.hubStart));

    SmartDashboard.putData("SysID/DynamicTestF", routine.dynamic(Direction.kForward));
    SmartDashboard.putData("SysID/QuasistaticTestF", routine.quasistatic(Direction.kForward));
    SmartDashboard.putData("SysID/DynamicTestR", routine.dynamic(Direction.kReverse));
    SmartDashboard.putData("SysID/QuasistaticTestR", routine.quasistatic(Direction.kReverse));
    SmartDashboard.putData("AntiJam Intake", new IntakeMoveAntiJam(intake, 1));
  }

  /**
   * Use this method to change LED's on Robot based on things happening during the match. I know,
   * I'm great at documentation :)
   */
  // TODO: Actually test this to make sure it works correctly
  private void configureDriverFeedback() {
    led.setDefaultCommand(new LEDToControlMode(this.led));

    // turns purple when the shooter is active, but not at the max RPM
    new Trigger(shooter::notMaxRPM)
        .onTrue(
            Commands.run(
                    () -> led.setPattern(LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.1))),
                    led)
                .withTimeout(1.5));
    // turns green when the shooter is active and at the max RPM
    new Trigger(shooter::atMaxRPM)
        .onTrue(
            Commands.run(
                    () -> led.setPattern(LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.1))),
                    led)
                .withTimeout(1.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.get();
  }
}
