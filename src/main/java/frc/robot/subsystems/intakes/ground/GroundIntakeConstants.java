package frc.robot.subsystems.intakes.ground;

public class GroundIntakeConstants {
  // wrist
  public static int wirstSparkCanId = 999999;
  public static boolean wirstSparkInverted = false;
  public static int wristCurrentLimit = 40;
  public static double wristSetpointTolerance = 0.02;

  // intake
  public static double intakeSparkCanId = 999998;
  public static boolean intakeSparkInverted = false;
  public static int intakeCurrentLimit = 40;
  public static double IntakeOutput = 1;

  public static int encoderPort = 0;

  public static double wristP = 0.0;
  public static double wristI = 0.0;
  public static double wristD = 0.0;

  public static double wristMinPosition = 0.0;
  public static double wristMaxPosition = 0.0;

  public static double stowedPosition = 0.0;
  public static double deployedPosition = 0.0;
}
