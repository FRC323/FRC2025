package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int elevatorPortLead = 1;
  public static int elevatorPortFollower = 2;

  public static double kP = 4.5;
  public static double kI = 0.0;
  public static double kD = 0.08;

  public static double HomeInches = 0.0;
  // reef levels
  public static double Level1Inches = 18.2;
  public static double Level2Inches = 32.0;
  public static double Level3Inches = 48.0;
  public static double Level4Inches = 72.0;

  public static enum ReefLevel {
    Level1,
    Level2,
    Level3,
    Level4
  }

  // https://www.reca.lc/linear
  public static double kS = 0.0;
  public static double kG = 0.52;
  public static double kV = 1.53;
  public static double kA = 0.05;

  public static double kDt = 0.02;
  public static double maxVelocity = Units.degreesToRadians(2048); // m/s
  public static double maxAcceleration = Units.degreesToRadians(18096); // m/s^2

  public static double gearRatio = 10;
  public static double carriageMass = 2.26; // kg
  public static double drumRadius = Units.inchesToMeters(2);
  public static double drumCircumference = Math.PI * drumRadius;
  public static double minElevatorHeightInches = 0; // inches
  public static double maxElevatorHeightInches = 60; // inches
}
