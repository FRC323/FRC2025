package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean leadSparkConnected = false;

    public double currentPositionRadians = 0.0;
    public double targetPositionRadians = 0.0;
    public boolean reachedDesiredPosition = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setOpenLoop(double output) {}

  public default void setAngleRadians(double targetAngleRadians) {}
}
