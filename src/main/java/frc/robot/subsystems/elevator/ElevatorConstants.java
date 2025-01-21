package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public static int elevatorPortLead = 1;
  public static int elevatorPortFollower = 2;

  public static double kDt = 0.02;
  public static double maxVelocity = 1.75;
  public static double maxAcceleration = 0.75;
  public static double kP = 0.005;
  public static double kI = 0.0;
  public static double kD = 0.0;

  // https://www.reca.lc/linear
  public static double kS = 0.0;
  public static double kG = 0.52;
  public static double kV = 3.07;
  public static double kA = 0.0;

  public static double gearRatio = 12.0 / 1.0;
  public static double carriageMass = 4; // kg
  public static double drumRadius = Units.inchesToMeters(2);
  public static double minElevatorHeightMeters = Units.inchesToMeters(0);
  public static double maxElevatorHeightMeters = Units.inchesToMeters(60); // inches
  /*

  gearRatio = 10.0 / 1.0;
  pulleyDiameterInches = 1.25;
  pulleyCircumferenceInches = pulleyDiameterInches * Math.PI;
  ticksToInches = 3.0 * gearRatio * pulleyCircumferenceInches / 8192.0;


  gearRatio = 12.0 / 1.0;
  pulleyDiameterInches = 1.25;
  pulleyCircumferenceInches = pulleyDiameterInches * Math.PI;
  encoderCPR = 8192.0;
  stages = 3.0;

  ticksToInches = (pulleyCircumferenceInches * stages) / (encoderCPR * gearRatio);
   */
}
