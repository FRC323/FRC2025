package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int leadCanId = 41;
  public static int followerCanId = 42;

  public static boolean leadInverted = false;
  public static boolean followerInverted = true;

  public static double kP = 1.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double HomePosition = 0.0;
  // reef levels
  public static double Level1Position = 10;
  public static double Level2Position = 20;
  public static double Level3Position = 30;
  public static double Level4Position = 40;

  public static int currentLimit = 40;

  public static enum ReefLevel {
    Level1,
    Level2,
    Level3,
    Level4
  }

  // https://www.reca.lc/linear
  public static double kS = 0.0;
  public static double kG = 0.9;
  public static double kV = 9.21;
  public static double kA = 0.01;

  public static double kDt = 0.02;
  // public static double maxVelocity = 1.30; // m/s;
  //  public static double maxAcceleration = 2.60; // m/s^2;

  public static double gearRatio = 12;
  public static double carriageMass = Units.lbsToKilograms(5);
  public static double drumRadiusInches = 2;
  public static double drumRadius = Units.inchesToMeters(2);
  public static double drumCircumference = Math.PI * drumRadius;
  public static double minElevatorHeight = 0; // unit = encoder position
  public static double maxElevatorHeight = 60; // unit = encoder position
}
