package org.jmhsrobotics.frc2026.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Preferences;

/**
 * Utility class for computing and persisting camera-to-robot calibration data.
 *
 * <p>Camera positions are stored in WPILib {@link Preferences} (NetworkTables-backed key/value
 * store) so they survive robot reboots and can be edited live from the Driver Station or
 * Shuffleboard without redeploying code.
 */
public class VisonPosition {

  /**
   * Given a raw calibration {@link Transform3d} (measured offset from the calibration rig origin to
   * the camera) and the known position of the calibration rig origin relative to the robot center,
   * compute the camera's {@link Transform3d} relative to the robot center.
   *
   * @param calibrationOffset known pose of the calibration rig origin in robot-center frame
   * @param calibration raw transform produced by the calibration procedure
   * @return camera-to-robot {@link Transform3d}
   */
  public static Transform3d computeRobotToCamera(
      Pose3d calibrationOffset, Transform3d calibration) {
    var cameraFromRobot = calibrationOffset.transformBy(calibration.inverse());
    return new Transform3d(cameraFromRobot.getTranslation(), cameraFromRobot.getRotation());
  }

  /**
   * Save a camera's robot-relative {@link Tranform} to {@link Preferences}.
   *
   * <p>Call this after a successful calibration run so the values persist across reboots.
   *
   * @param name unique camera name (e.g. {@code "blackbird"})
   * @param cameraPos camera pose in robot-center frame
   */
  public static void saveCameraConfig(String name, Transform3d cameraPos) {
    String posKey = "camCal/" + name + "/pos/";
    Preferences.setDouble(posKey + "x", cameraPos.getX());
    Preferences.setDouble(posKey + "y", cameraPos.getY());
    Preferences.setDouble(posKey + "z", cameraPos.getZ());

    Quaternion rot = cameraPos.getRotation().getQuaternion();
    String rotKey = "camCal/" + name + "/rot/";
    Preferences.setDouble(rotKey + "w", rot.getW());
    Preferences.setDouble(rotKey + "x", rot.getX());
    Preferences.setDouble(rotKey + "y", rot.getY());
    Preferences.setDouble(rotKey + "z", rot.getZ());
  }

  /**
   * Load a camera's robot-relative {@link Pose3d} from {@link Preferences}.
   *
   * <p>If a key does not yet exist in Preferences, the supplied {@code defaultPose} value is
   * written back so it becomes visible (and editable) in Shuffleboard / the Driver Station.
   *
   * @param name unique camera name (e.g. {@code "blackbird"})
   * @param defaultPose pose to use (and persist) when no saved value exists yet
   * @return the loaded (or default) camera pose in robot-center frame
   */
  public static Transform3d loadCameraConfig(String name, Transform3d defaultPose) {
    String posKey = "camCal/" + name + "/pos/";
    String rotKey = "camCal/" + name + "/rot/";

    // Initialise keys with defaults on first run so they appear in Shuffleboard
    Quaternion defaultQuat = defaultPose.getRotation().getQuaternion();
    Preferences.initDouble(posKey + "x", defaultPose.getX());
    Preferences.initDouble(posKey + "y", defaultPose.getY());
    Preferences.initDouble(posKey + "z", defaultPose.getZ());
    Preferences.initDouble(rotKey + "w", defaultQuat.getW());
    Preferences.initDouble(rotKey + "x", defaultQuat.getX());
    Preferences.initDouble(rotKey + "y", defaultQuat.getY());
    Preferences.initDouble(rotKey + "z", defaultQuat.getZ());

    // Read back whatever is currently stored (may be the just-written defaults)
    double px = Preferences.getDouble(posKey + "x", defaultPose.getX());
    double py = Preferences.getDouble(posKey + "y", defaultPose.getY());
    double pz = Preferences.getDouble(posKey + "z", defaultPose.getZ());

    double qw = Preferences.getDouble(rotKey + "w", defaultQuat.getW());
    double qx = Preferences.getDouble(rotKey + "x", defaultQuat.getX());
    double qy = Preferences.getDouble(rotKey + "y", defaultQuat.getY());
    double qz = Preferences.getDouble(rotKey + "z", defaultQuat.getZ());

    return new Transform3d(px, py, pz, new Rotation3d(new Quaternion(qw, qx, qy, qz)));
  }

  public static Transform3d loadCameraConfig(String name) {
    return loadCameraConfig(name, new Transform3d());
  }
}
