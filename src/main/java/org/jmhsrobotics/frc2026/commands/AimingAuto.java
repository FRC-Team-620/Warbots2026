package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.jmhsrobotics.frc2026.Constants;
import org.jmhsrobotics.frc2026.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.indexer.Indexer;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;
import org.jmhsrobotics.frc2026.subsystems.vision.Vision;

public class AimingAuto extends SequentialCommandGroup {
  private Drive drive;
  private Shooter shooter;
  private Vision vision;

  // x - 3.642  y - 0.452
  // feeder removed (2026-07-19): dropped the Feeder param since RobotContainer no
  // longer constructs one — see RobotContainer.java for the full note.
  public AimingAuto(
      Drive drive, Shooter shooter, Indexer indexer, Pose2d pose, ControlBoard control) {
    // this.drive = drive;
    // this.shooter = shooter;

    // addRequirements(drive, shooter);
    addCommands(
        new SequentialCommandGroup(
            Commands.runOnce(() -> drive.setPose(pose), drive),
            new ParallelCommandGroup(
                new AlignToHub(drive, control),
                new DistanceAdjustingShoot(shooter, drive),
                new IndexerMove(indexer, Constants.Indexer.kSpeedDutyCycle)
                // new Feed(feeder, Constants.Feeder.kSpeedDutyCycle, shooter)
                )));
  }
}
