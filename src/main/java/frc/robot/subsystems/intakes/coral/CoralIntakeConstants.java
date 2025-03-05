package frc.robot.subsystems.intakes.coral;

public class CoralIntakeConstants {
  public static int spark1CanId = 51;
  public static int spark2CanId = 52;

  public static boolean spark1Inverted = false;
  public static boolean spark2Inverted = true;

  public static int currentLimit = 5;

  public static double normalOutput = .3;
  public static double IntakeOutput = -1;

  public static double capturedCurrentOutput = 15;
  // output to 'carry' game piece
  // public static double capturedCurrentOutput = .15;
  // the current at which we detect a game piece being caputured
  // public static double capturingPieceCurrent = 25;
  // the amount of time we want to 'watch' high current for
  // public static double currentSpikeDuration = 1.0;
}
