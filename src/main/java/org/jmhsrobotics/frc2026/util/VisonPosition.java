package org.jmhsrobotics.frc2026.util;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Preferences;

public class VisonPosition {
    public static final Pose3d OFFSET = new Pose3d(1, 0, 0.25, new Rotation3d(Rotation2d.fromDegrees(180)));

    public static Transform3d setCalibration(double x, double y, double z, Quaternion quaternion) {
        var calibration = new Transform3d(
                x,
                y,
                z,
                new Rotation3d(quaternion));

        var calibrationOffset =  OFFSET.transformBy(calibration.inverse());

        return new Transform3d(calibrationOffset.getTranslation(), calibrationOffset.getRotation());
    }

    //TODO: Add Javadocs
    public static Pose3d computeCameraPositionFromTarget(Pose3d calTargetFromRobot, Transform3d measuredTagPos){
        // Given a Known tag location from the center of the robot. IE a calibration rig, and the measured (Camera Rel) position of the april tag
        // Compute the camera's position relitive to the robot's center as well as its rotation
        var cameraPos = calTargetFromRobot.transformBy(measuredTagPos.inverse());
        return cameraPos;
    }

    private static void saveCameraConfig(String name,Pose3d cameraPos){
        Preferences.initDouble("camCal/" + name + "/", cameraPos.getX());
    }

}
