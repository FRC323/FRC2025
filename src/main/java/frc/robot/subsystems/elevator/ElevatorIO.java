package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leadSparkConnected = false;
    public boolean followerSparkConnected = false;

    public double ElevatorHeightInches = 0.0;
    public double ElevatorTargetHeightInches = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setHeightInches(double targetHeightInches) {}
}
