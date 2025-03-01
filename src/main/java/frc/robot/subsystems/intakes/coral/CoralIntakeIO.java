package frc.robot.subsystems.intakes.coral;

import frc.robot.subsystems.intakes.coral.CoralIntake.IntakeState;
import org.littletonrobotics.junction.AutoLog;

public interface CoralIntakeIO {
  @AutoLog
  public static class CoralIntakeIOInputs {
    public boolean spark1SparkConnected = false;
    public boolean spark2SparkConnected = false;

    public double spark1OutputCurrent = 0.0;
    public double spark2OutputCurrent = 0.0;

    public double spark1SpeedPercent = 0.0;
    public double spark2SpeedPercent = 0.0;

    public boolean spark1Engaged = false;
    public boolean spark2Engaged = false;

    public double totalOutputCurrent = 0.0;

    public IntakeState state = IntakeState.IDLE;
  }

  public default void setPercent(double percent) {}

  public default void updateInputs(CoralIntakeIOInputs inputs) {}
}
