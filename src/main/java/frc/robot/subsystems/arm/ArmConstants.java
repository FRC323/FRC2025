package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
  public static final int leadCanId = 31;

  public static final boolean leadInverted = false;

  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static String ArmOffsetKey = "arm_offset";

  public static double ArmLengthMeters = Units.inchesToMeters(36.0);

  public static double HomePosition = 0.0;

  public static double ReefLevel1CoralPosition = 3.92;
  public static double ReefLevel2CoralPosition = 3.92;
  public static double ReefLevel3CoralPosition = 3.92;
  public static double ReefLevel4CoralPosition = 3.92;

  public static double HumanPlayerPosition = 0.79;

  public static double ReefLevel1AlgaePosition = 2.35;
  public static double ReefLevel2AlgaePosition = 2.35;

  public static double BargePosition = -0.71;
  public static double ProcessorPosition = 0.76;

  public static int currentLimit = 20;

  public static final double kS = 0.0;
  public static final double kG = 0.89;
  public static final double kV = 2.44;
  public static final double kA = 0.04;

  public static double kDt = 0.02;

  public static final double maxVelocity = 10.0;
  public static final double maxAcceleration = 5.0;

  public static final double gearRatio = 125.0;

  public static final double maxEncoderDrift = .05;
  public static final double minAngleRadians = Units.degreesToRadians(0);
  public static final double maxAngleRadians = Units.degreesToRadians(270);

  public static final double SetpointTolerance = 0.05;
}
