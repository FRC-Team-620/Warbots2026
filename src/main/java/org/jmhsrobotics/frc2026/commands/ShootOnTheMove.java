package org.jmhsrobotics.frc2026.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.jmhsrobotics.frc2026.FieldConstants;
import org.jmhsrobotics.frc2026.controlBoard.ControlBoard;
import org.jmhsrobotics.frc2026.fireControl.ShotCalculator;
import org.jmhsrobotics.frc2026.subsystems.drive.Drive;
import org.jmhsrobotics.frc2026.subsystems.shooter.Shooter;
import org.jmhsrobotics.frc2026.subsystems.vision.Vision;

public class ShootOnTheMove extends Command {

  private final Drive drive;
  private final Vision vision;
  private final Shooter shooter;

  private final ShotCalculator m_shotCalc = new ShotCalculator();
  private final ControlBoard control;

  public ShootOnTheMove(Drive drive, Vision vision, Shooter shooter, ControlBoard control) {
    this.drive = drive;
    this.vision = vision;
    this.shooter = shooter;
    this.control = control;
  }

  @Override
  public void initialize() {}

  public void execute() {
    // In a command's execute() or in Robot.robotPeriodic()
    Translation2d centerHub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    ; // or whatever your FieldConstants uses for the target
    Translation2d hubForward = new Translation2d(1, 0); // target orientation (check FieldConstants)

    ShotCalculator.ShotInputs inputs =
        new ShotCalculator.ShotInputs(
            drive.getPose(), // from subsystems.drive.Drive.java
            ChassisSpeeds.fromFieldRelativeSpeeds(
                drive.getChassisSpeeds(),
                drive.getRotation()), // or convert ChassisSpeeds to field frame if needed
            drive.getChassisSpeeds(), // robot-relative velocity (check Drive.java)
            centerHub,
            hubForward,
            vision.getTargetConfidence(), // from subsystems.vision/ (0.0–1.0)
            drive.getPitchDegrees(), // from GyroIOPigeon2 / GyroIO in drive/
            drive.getRollDegrees());

    ShotCalculator.LaunchParameters shot = m_shotCalc.calculate(inputs);

    if (shot.isValid() && shot.confidence() > 50) {
      // Command the shooter
      shooter.setRPM(shot.rpm());

      // Command the drive to aim (this is the "on the move" part)
      new AlignToHub(drive, control);
    }
  }
}
