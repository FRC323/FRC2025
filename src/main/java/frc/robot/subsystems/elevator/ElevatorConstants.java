package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int leadCanId = 41;
  public static int followerCanId = 42;

  public static boolean leadInverted = true;
  public static boolean followerInverted = true;

  public static double kP = 0.14;
  public static double kI = 0.0045;
  public static double kD = 0.002;

  public static double HomePosition = 0.0;
  public static double TravelPosition = 3.50;
  // reef levels
  public static double ReefLevel1CoralPosition = 8.42;
  public static double ReefLevel2CoralPosition = 16.47; // 5.64;
  public static double ReefLevel3CoralPosition = 30.47;
  public static double ReefLevel4CoralPosition = 55.00;

  public static double ReefLevel1AlgaePosition = 17.57;
  public static double ReefLevel2AlgaePosition = 32.07;

  public static double HumanPlayerPosition = 2.00;

  public static double BargePosition = 54.00;
  public static double ProcessorPosition = 0.0;

  public static int currentLimit = 40;

  // https://www.reca.lc/linear
  // public static double kS = 0.0;
  // public static double kG = 0.26;
  // public static double kV = 9.21;
  // public static double kA = 0.03;

  public static double kDt = 0.02;
  // public static final double TUNING_MODE_MULTIPLIER = 1;
  // public static double maxVelocity = 140 * TUNING_MODE_MULTIPLIER; // 140.93 m/s;
  // public static double maxAcceleration = 95 * TUNING_MODE_MULTIPLIER; // 211.39 m/s^2;

  public static double gearRatio = 12;
  public static double carriageMass = Units.lbsToKilograms(15);
  public static double drumRadius = Units.inchesToMeters(2);
  public static double drumCircumference = Math.PI * drumRadius;
  public static double minElevatorHeight = 0; // unit = encoder position
  public static double maxElevatorHeight = 60; // unit = encoder position

  public static double minRotations = 0;
  public static double maxRotations = 0;
}
