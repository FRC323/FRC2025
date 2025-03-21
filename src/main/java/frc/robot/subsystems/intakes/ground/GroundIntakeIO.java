package frc.robot.subsystems.intakes.ground;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
  @AutoLog
  public static class GroundIntakeIOInputs {
    public boolean wristSparkConnected = false;
    public double wristOutputCurrent = 0.0;
    public double wristTargetPosition = 0.0;
    public double wristCurrentPosition = 0.0;

    public boolean intakeSparkConnected = false;
    public double intakeOutputCurrent = 0.0;
  }

  public default double getWristPosition() {
    return 0.0;
  }

  public default void setWristPercent(double percent) {}

  public default void setWristVoltage(double voltage) {}

  public default void setWristPosition(double position) {}

  public default void setIntakePercent(double percent) {}

  public default void updateInputs(GroundIntakeIOInputs inputs) {}
}
