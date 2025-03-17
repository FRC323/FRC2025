package frc.robot.util;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

public class LaserCanUtil {
  public static Measurement getMeasurement(LaserCan laser) {
    var measurement = laser.getMeasurement();
    if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return null;
    }
    return measurement;
  }

  public static double getDistanceMM(LaserCan laserCan) {
    var measurement = getMeasurement(laserCan);
    if (measurement == null) {
      return Double.NaN;
    }
    return measurement.distance_mm;
  }
}
