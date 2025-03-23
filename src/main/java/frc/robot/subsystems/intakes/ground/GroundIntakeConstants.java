package frc.robot.subsystems.intakes.ground;

public class GroundIntakeConstants {
  // wrist
  public static int wirstSparkCanId = 26;
  public static boolean wirstSparkInverted = false;
  public static int wristCurrentLimit = 40;
  public static double wristSetpointTolerance = 0.01;

  // intake
  public static int intakeSparkCanId = 25;
  public static boolean intakeSparkInverted = false;
  public static int intakeCurrentLimit = 40;
  public static double IntakeOutput = -1;

  public static double wristP = 4.5;
  public static double wristI = 0.0;
  public static double wristD = 0.0;

  public static double wristMinPosition = 0.150;
  public static double wristMaxPosition = 0.481;

  public static double stowedPosition = 0.138;
  public static double deployedPosition = 0.481;
}
