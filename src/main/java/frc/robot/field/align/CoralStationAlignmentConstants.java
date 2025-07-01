package frc.robot.field.align;

import java.util.HashMap;

public class CoralStationAlignmentConstants {
  public static final String smartDashboardLogPrefix = "CStationAlign/";

  public static final double xyScalingFactor = 1.0;
  public static final double rotationScalingFactor = 1.0;

  public static final double outwardOffsetFromTag = 0.1;
  public static final double lateralOffsetFromTag = 0.0;

  public static final boolean moveRobot = true;
  public static final boolean smartDashboardLogging = false;

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
