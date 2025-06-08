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

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.HashMap;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public final class ReefAlignmentConstants {
    public static final String smartDashboardLogPrefix = "ReefAlign/";

    public static final double depthP = 2;
    public static final double depthI = 0.0;
    public static final double depthD = 0.0;
    public static final double depthFF = 0.0;
    public static final double depthThreshold = 0.05;

    public static final double lateralP = 2;
    public static final double lateralI = 0.0;
    public static final double lateralD = 0.0;
    public static final double lateralThreshold = 0.05;

    public static final double rotationP = 10;
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.0;
    public static final double rotationFF = 0.0;
    public static final double rotationThreshold = 0.05;

    public static final double maxVelocity = 1.0;

    public static final double tagScanTimeoutInSeconds = 2.0;

    // robot width / 2
    public static final double outwardOffsetFromTag = 0.38;

    // center of the reef translation - blue
    public static final Translation2d blueReefCenter = new Translation2d(4.49, 4.02);
    // center of the reef translation - red
    public static final Translation2d redReefCenter = new Translation2d(13.03, 4.05);

    public static final boolean moveRobot = true;

    public static final HashMap<Integer, Alliance> tagToAlliance =
        new HashMap<>() {
          {
            put(17, Alliance.Blue); // blue
            put(18, Alliance.Blue);
            put(19, Alliance.Blue);
            put(20, Alliance.Blue);
            put(21, Alliance.Blue);
            put(22, Alliance.Blue);

            put(6, Alliance.Red); // red
            put(7, Alliance.Red);
            put(8, Alliance.Red);
            put(9, Alliance.Red);
            put(10, Alliance.Red);
            put(11, Alliance.Red);
          }
        };

    // offset from the tag to the pole in meters
    public static final HashMap<String, Double> tagToPoleOffset =
        new HashMap<>() {
          {
            put("17_left", .165); // blue
            put("17_right", .165);
            put("18_left", .165);
            put("18_right", .165);
            put("19_left", .165);
            put("19_right", .165);
            put("20_left", .165);
            put("20_right", .165);
            put("21_left", .165);
            put("21_right", .165);
            put("22_left", .250);
            put("22_right", .250);

            put("6_left", .165); // red
            put("6_right", .165);
            put("7_left", .165);
            put("7_right", .165);
            put("8_left", .165);
            put("8_right", .165);
            put("9_left", .165);
            put("8_right", .165);
            put("10_left", .165);
            put("10_right", .165);
            put("11_left", .165);
            put("11_right", .165);
          }
        };

    // rotation target for the robot to align with the reef
    public static final HashMap<Integer, Double> tagToRotationTarget =
        new HashMap<Integer, Double>() {
          {
            put(17, -30.0); // blue
            put(18, -90.0);
            put(19, -150.0);
            put(20, 150.0);
            put(21, 90.0);
            put(22, 30.0);

            put(6, 30.0); // red
            put(7, 90.0);
            put(8, 150.0);
            put(9, -150.0);
            put(10, -90.0);
            put(11, -30.0);
          }
        };

    public static Pose3d tag22AlignInitPose =
        new Pose3d(5.69, 2.10, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(30.0)));

    public static Pose3d tag17AlignInitPose =
        new Pose3d(2.85, 2.54, 0.0, new Rotation3d(0.0, 0.0, Math.toRadians(-62.7)));
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class Robot {}
}
