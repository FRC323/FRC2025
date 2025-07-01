package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean leadSparkConnected = false;
    public boolean followerSparkConnected = false;

    public boolean homed = false;
    public boolean isAtBottom = false;

    public double leadEncoderPosition = 0.0;
    public double followerEncoderPosition = 0.0;

    public double leadEncoderVelocity = 0.0;
    public double followerEncoderVelocity = 0.0;
  }

  public default void setPercent(double percent) {}

  public default void setVoltage(double voltage) {}

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setPosition(double targetPosition) {}
}
