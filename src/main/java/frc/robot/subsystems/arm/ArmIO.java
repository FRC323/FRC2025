package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean leadSparkConnected = false;

    public double currentAbsolutePositionRadians = 0.0;
    public double currentRelativePositionRadians = 0.0;
    public double targetPositionRadians = 0.0;
    public boolean reachedDesiredPosition = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void setVoltage(double voltage) {}

  public default void setAngleRadians(double targetAngleRadians) {}

  public default void storeOffset() {}
}
