package frc.robot.field.align;

import java.util.HashMap;

public class CoralStationAlignmentConstants {
  public static final String smartDashboardLogPrefix = "CStationAlign/";
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

  public static final double xyScalingFactor = 1.0;
  public static final double rotationScalingFactor = 1.0;

  public static final double outwardOffsetFromTag = 0.42;
  public static final double lateralOffsetFromTag = 0.0;

  public static final boolean moveRobot = true;
  public static final boolean smartDashboardLogging = true;

  public static final HashMap<Integer, Double> tagToRotationTarget =
      new HashMap<Integer, Double>() {
        {
          put(12, -35.0); // blue
          put(13, -145.0);

          put(1, 35.0); // red
          put(2, 145.0);
        }
      };
}
