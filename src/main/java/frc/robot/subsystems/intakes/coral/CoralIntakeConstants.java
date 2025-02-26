package frc.robot.subsystems.intakes.coral;

public class CoralIntakeConstants {
  public static int spark1CanId = 51;
  public static int spark2CanId = 52;

  public static boolean spark1Inverted = false;
  public static boolean spark2Inverted = true;

  public static int currentLimit = 40;

  // normal output when intake is waiting for game piece
  public static double normalOutput = .75;
  // output to 'carry' game piece
  public static double capturedCurrentOutput = .15;
  // the current at which we detect a game piece being caputured
  public static double capturingPieceCurrent = 0.0;
}
