package frc.robot.subsystems.intakes.coral;

public class CoralIntakeConstants {
  public static int spark1CanId = 51;
  public static int spark2CanId = 52;

  public static boolean spark1Inverted = true;
  public static boolean spark2Inverted = false;

  public static int currentLimit = 15;

  public static double normalOutput = .5;
  public static double coralArmInvertedOutput = .75;
  public static double IntakeOutput = -1;

  public static int laserCanId = 60;
  // reef branch min - end of coral in intake
  // reef branch max - from end of coral intake to branch
  public static double reefBranchDistanceMin = 200.0;
  public static double reefBranchDistanceMax = 380.0;
  public static double reefBranchDistanceTolerance = 0.05;

  public static double capturedCurrentOutput = 15;
  // output to 'carry' game piece
  // public static double capturedCurrentOutput = .15;
  // the current at which we detect a game piece being caputured
  // public static double capturingPieceCurrent = 25;
  // the amount of time we want to 'watch' high current for
  // public static double currentSpikeDuration = 1.0;
}
