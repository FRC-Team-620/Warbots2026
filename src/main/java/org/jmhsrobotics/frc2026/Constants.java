// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jmhsrobotics.frc2026;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final double ksimTimestep = 0.02;
  public static final double krealTimeStep = ksimTimestep;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class LEDConstants {
    public static final int kPWMHeader = 5;
    // led length in pixels
    public static final int kLength = 150;
    // Density of the LED Strip - currently set at 120 LEDs per meter
    public static final Distance kSpacing = Meters.of(1.0 / 60);
    // number of times the flashcommand will change color per second
    public static final double kFlashFrequency = 10;
  }

  public static class CAN {
    // TODO update CANIds
    public static final int kCanAndGyroID = 60;
    public static final int kFlywheelMotorID = 55;
    public static final int kIntakeMotorID = 50;
    public static final int kSlapDownMotorID = 51;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ShooterConstants {

    public static final double kP = 0.008;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kBaseRPM = 3000;
  }

  public static class Intake {
    public static final double kIntakeP = 0.0;
    public static final double kIntakeI = 0.0;
    public static final double kIntakeD = 0.0;
    public static final double kSlapdownP = 0.0;
    public static final double kSlapdownI = 0.0;
    public static final double kSlapdownD = 0.0;

    public static final double kBaseRPM = 1000;
    public static final double kSlapDownUpPositionDegrees = 0; // TODO update this (IMPORTANT)
    public static final double kSlapDownDownPositionDegrees = 0; // TODO update this (IMPORTANT)
    public static final double kSlapDownToleranceDegrees = 5; // TODO update this (IMPORTANT)
  }
}
