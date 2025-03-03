package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean leadSparkConnected = false;

    public double currentAbsolutePosition = 0.0;
    public double currentRelativePosition = 0.0;
    public double currentVelocity = 0.0;
    public double targetPosition = 0.0;
    public boolean reachedDesiredPosition = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void setVoltage(double voltage) {}

  public default void setAngleRadians(double targetAngle) {}
}
