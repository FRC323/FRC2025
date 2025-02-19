package frc.robot.subsystems.climber;

import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean leadSparkConnected = false;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void setVoltage(double voltage) {}

  public default void setAngleRadians(double targetAngleRadians) {}
}
