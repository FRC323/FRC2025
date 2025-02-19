package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int leadCanId = 41;
  public static int followerCanId = 42;

  public static boolean leadInverted = true;
  public static boolean followerInverted = true;

  public static double kP = 1.0;
  public static double kI = 0.0;
  public static double kD = 0.0;

  public static double HomePosition = 0.0;
  // reef levels
  public static double ReefLevel1CoralPosition = 10;
  public static double ReefLevel2CoralPosition = 20;
  public static double ReefLevel3CoralPosition = 30;
  public static double ReefLevel4CoralPosition = 40;

  public static double ReefLevel1AlgaePosition = 15;
  public static double ReefLevel2AlgaePosition = 25;

  public static double HumanPlayerPosition = 0.0;

  public static double BargePosition = 60;
  public static double ProcessorPosition = 0.0;

  public static int currentLimit = 40;

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
