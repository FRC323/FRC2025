package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean leadSparkConnected = false;

    public double targetPositionRadians = 0.0;
    public boolean reachedDesiredPosition = false;

    public ClimberState currentState = ClimberState.STOWED;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void setVoltage(double voltage) {}
}
