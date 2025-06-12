package frc.robot.field.align;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.ReefPoleOffset;
import java.util.HashMap;

public final class ReefAlignmentConstants {
  public static final String smartDashboardLogPrefix = "ReefAlign/";

  public enum PoleSide {
    LEFT,
    RIGHT
  }

  public enum ReefPoleLabel {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L,
  }

  public static final double depthP = 2;
  public static final double depthI = 0.0;
  public static final double depthD = 0.0;
  public static final double depthFF = 0.0;

  public static final double lateralP = 2;
  public static final double lateralI = 0.0;
  public static final double lateralD = 0.0;

  public static final double rotationP = 10;
  public static final double rotationI = 0.0;
  public static final double rotationD = 0.0;
  public static final double rotationFF = 0.0;

  public static final double xTolerance = 0.05;
  public static final double yTolerance = 0.05;
  public static final double rotationTolerance = 2.0;

  public static final double rotationMaxVelocity = 3.0; // m/s
  public static final double rotationMaxAcceleration = 3.0; // m/s^2

  public static final double tagScanTimeoutInSeconds = 2.0;

  public static final double xyScalingFactor = 1.0;
  public static final double rotationScalingFactor = 1.0;

  // robot width / 2
  public static final double outwardOffsetFromTag = 0.42;

  // center of the reef translation - blue
  public static final Translation2d blueReefCenter = new Translation2d(4.49, 4.02);
  // center of the reef translation - red
  public static final Translation2d redReefCenter = new Translation2d(13.03, 4.05);

  public static final boolean moveRobot = true;
  public static final boolean smartDashboardLogging = true;

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
  public static final HashMap<Integer, ReefPoleOffset> tagToPoleOffset =
      new HashMap<>() {
        {
          put(17, new ReefPoleOffset(.165, .165)); // blue
          put(18, new ReefPoleOffset(.165, .165));
          put(19, new ReefPoleOffset(.165, .165));
          put(20, new ReefPoleOffset(.165, .165));
          put(21, new ReefPoleOffset(.165, .165));
          put(22, new ReefPoleOffset(.165, .165));

          put(6, new ReefPoleOffset(.165, .165)); // red
          put(7, new ReefPoleOffset(.165, .165));
          put(8, new ReefPoleOffset(.165, .165));
          put(9, new ReefPoleOffset(.165, .165));
          put(10, new ReefPoleOffset(.165, .165));
          put(11, new ReefPoleOffset(.165, .165));
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
