package frc.robot.util;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public class LaserCanUtil {
  public static Measurement getMeasurement(LaserCan laser) {
    var measurement = laser.getMeasurement();
    if (measurement == null) {
      return null;
    }
    return measurement;
  }

  public static double getDistanceMM(LaserCan laserCan) {
    var measurement = getMeasurement(laserCan);
    if (measurement == null) {
      return 399;
    }
    return measurement.distance_mm;
  }
}
