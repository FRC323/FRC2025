package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int leadCanId = 31;

  public static final boolean leadInverted = false;

  public static final double kP = 2.5; // 13.0;
  public static final double kI = 0.01; // 0.024;
  public static final double kD = 0.0; // 0.002;

  public static String ArmOffsetKey = "arm_offset";
  public static double ArmLengthMeters = Units.inchesToMeters(36.0);

  public static double HomePosition = 0.076;

  public static double ReefLevel1CoralPosition = .66;
  public static double ReefLevel2CoralPosition = .66;
  public static double ReefLevel3CoralPosition = .66;
  public static double ReefLevel4CoralPosition = .71;

  public static double HumanPlayerPosition = 0.159;

  public static double ReefLevel1AlgaePosition = .265;
  public static double ReefLevel2AlgaePosition = .265;

  public static double BargePosition = 0;
  public static double ProcessorPosition = 0;

  public static int currentLimit = 40;

  public static final double kS = 0.0;
  public static final double kG = 0.41;
  public static final double kV = 5.26;
  public static final double kA = 0.02;

  public static double kDt = 0.02;

  public static final double maxVelocity = 0.3;
  public static final double maxAcceleration = .3;

  public static final double gearRatio = 270;

  public static final double maxEncoderDrift = .05;
  public static final double minPosition = 0;
  public static final double maxPosition = .75;

  public static final double SetpointTolerance = 0.05;
}

// 270
