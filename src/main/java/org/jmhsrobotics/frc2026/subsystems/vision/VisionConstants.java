// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.jmhsrobotics.frc2026.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  // TODO: Confirm and Identify which camera is on which side, update camera names
  public static String camera0Name = "rex";
  public static String camera1Name = "fives";
  public static String camera2Name = "camera2";
  public static String camera3Name = "Camera3";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // TODO: Make sure camera transformations are correctx
  //   public static Transform3d robotToCamera0 =
  //       new Transform3d(
  //           Units.inchesToMeters(8),
  //           Units.inchesToMeters(11),
  //           0.185,
  //           new Rotation3d(0.0, Units.degreesToRadians(-10), Units.degreesToRadians(-35)));

  //   public static Transform3d robotToCamera1 =
  //       new Transform3d(
  //           Units.inchesToMeters(8.5),
  //           Units.inchesToMeters(-11),
  //           0.185,
  //           new Rotatio(0.0, Units.degreesToRadians(-10), Units.degreesToRadians(35)));

  public static Transform3d rexCalibration =
      new Transform3d(
          new Translation3d(-0.32, 0.05, 0.35),
          new Rotation3d(
              new Quaternion(
                  0.25820065290841687,
                  -0.04437469554990406,
                  8.498465320544434E-4,
                  0.9650712859646134)));
  public static Transform3d codyCalibration =
      new Transform3d(-0.308, 0.062, 0.343, new Rotation3d(0, 0, Units.degreesToRadians(-210)));
  // 0.795, -0.224, -0.139, new Rotation3d());
  // blackbirdCalibration = new Pose3d();
  // TODO: Make sure all cameras are calibrated correctly
  public static Transform3d fivesCalibration =
      new Transform3d(-0.308, -0.062, 0.343, new Rotation3d(0, 0, Units.degreesToRadians(210)));
  public static Transform3d camera3Calibration =
      new Transform3d(
          0.781, -0.238, -0.178, new Rotation3d(new Quaternion(-0.303, -0.138, -0.020, -0.943)));

  public static Pose3d calibrationOffset =
      new Pose3d(1, 0, 0.25, new Rotation3d(Rotation2d.fromDegrees(180)));
  public static final Pose3d rexToRobot = calibrationOffset.transformBy(rexCalibration.inverse());
  public static final Pose3d codyToRobot = calibrationOffset.transformBy(codyCalibration.inverse());
  public static final Pose3d fivesToRobot =
      calibrationOffset.transformBy(fivesCalibration.inverse());
  public static final Pose3d camera3ToRobot =
      calibrationOffset.transformBy(camera3Calibration.inverse());

  public static final Transform3d robotToCamera0 = rexCalibration;
  public static final Transform3d robotToCamera1 = fivesCalibration;
  public static final Transform3d robotToCamera2 =
      new Transform3d(fivesToRobot.getTranslation(), fivesToRobot.getRotation());
  public static final Transform3d robotToCamera3 =
      new Transform3d(camera3ToRobot.getTranslation(), camera3ToRobot.getRotation());

  //   public static final Transform3d robotToCamera0 = blackbirdToRobot.plus(calibrationOffset);
  //   public static final Transform3d robotToCamera1 = overtureToRobot.plus(calibrationOffset);

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.1; // Meters
  public static double angularStdDevBaseline = 100; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0, // Camera 2
        1.0
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
