package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.Climber.ClimberPosition;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean leadSparkConnected = false;

    public double position = 0.0;
    public boolean reachedDesiredPosition = false;
    public double leadEncoderPosition = 0.0;
    public double leadEncoderVelocity = 0.0;
    public double CurrentOutput = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setPosition(ClimberPosition position) {}

  public default void setPercent(double percent, ClimberPosition position) {}

  public default void setVoltage(double voltage, ClimberPosition position) {}

  public default double getCurrentPosition() {
    return 0.0;
  }

  public default double getTargetPosition() {
    return 0.0;
  }

  public default void resetZero() {}

  public default void stop() {}
}
