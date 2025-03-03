package frc.robot.subsystems.climber;

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

  public default void setPercent(double percent) {}

  public default void setVoltage(double voltage) {}

  public default double getPosition() {
    return 0.0;
  }
}
