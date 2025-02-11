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
  private final ElevatorIOInputsAutoLogged ioInputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
    leadDisconnectedAlert = new Alert("Disconnected lead motor on elevator.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Disconnected follower motor on elevator.", AlertType.kError);
  }

  public void setHeight(double targetHeightInches) {
    targetHeightInches =
        MathUtil.clamp(
            targetHeightInches,
            ElevatorConstants.minElevatorHeightInches,
            ElevatorConstants.maxElevatorHeightInches);

    io.setHeightInches(targetHeightInches);
  }

  public void setHeight(ReefLevel reefLevel) {
    switch (reefLevel) {
      case Level1:
        setHeight(ElevatorConstants.Level1Inches);
        break;
      case Level2:
        setHeight(ElevatorConstants.Level2Inches);
        break;
      case Level3:
        setHeight(ElevatorConstants.Level3Inches);
        break;
      case Level4:
        setHeight(ElevatorConstants.Level4Inches);
        break;
    }
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(ioInputs.ElevatorTargetHeightInches - ioInputs.ElevatorHeightInches) < 0.2;
  }

  public double getHeightInInches() {
    return ioInputs.ElevatorHeightInches;
  }

  public double getTargetHeightInInches() {
    return ioInputs.ElevatorTargetHeightInches;
  }

  public void periodic() {
    io.updateInputs(ioInputs);
    Logger.processInputs("Elevator", ioInputs);

    leadDisconnectedAlert.set(!ioInputs.leadSparkConnected);
    followerDisconnectedAlert.set(!ioInputs.followerSparkConnected);
  }
}
