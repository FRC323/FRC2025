package frc.robot.subsystems.intakes.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIntakeIO {
  @AutoLog
  public static class AlgaeIntakeIOInputs {
    public boolean spark1SparkConnected = false;
    public boolean spark2SparkConnected = false;

    public double spark1OutputCurrent = 0.0;
    public double spark2OutputCurrent = 0.0;

    public double spark1SpeedPercent = 0.0;
    public double spark2SpeedPercent = 0.0;

    public boolean spark1Engaged = false;
    public boolean spark2Engaged = false;
  }

  public default void setPercent(double percent) {}

  public default void updateInputs(AlgaeIntakeIOInputs inputs) {}
}
