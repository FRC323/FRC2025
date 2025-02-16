package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefLevel;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final Alert leadDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
    leadDisconnectedAlert = new Alert("Disconnected lead motor on elevator.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Disconnected follower motor on elevator.", AlertType.kError);
  }

  public boolean isAtBottom() {
    return inputs.isAtBottom;
  }

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void runVoltageOutput(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    io.setVoltage(voltage);
  }

  public double getHeightInches() {
    return inputs.currentHeightPosition;
  }

  public void setHeight(double targetHeightPosition) {
    var position =
        MathUtil.clamp(
            targetHeightPosition,
            ElevatorConstants.minElevatorHeight,
            ElevatorConstants.maxElevatorHeight);

    io.setTargetHeight(position);
  }

  public void setHeight(ReefLevel reefLevel) {
    switch (reefLevel) {
      case Level1:
        setHeight(ElevatorConstants.Level1Position);
        break;
      case Level2:
        setHeight(ElevatorConstants.Level2Position);
        break;
      case Level3:
        setHeight(ElevatorConstants.Level3Position);
        break;
      case Level4:
        setHeight(ElevatorConstants.Level4Position);
        break;
    }
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(inputs.targetHeightPosition - inputs.currentHeightPosition) < 0.1;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
    followerDisconnectedAlert.set(!inputs.followerSparkConnected);
  }
}
