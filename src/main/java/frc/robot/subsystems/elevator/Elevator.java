package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final Alert leadDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged ioInputs = new ElevatorIOInputsAutoLogged();

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

  public Elevator(ElevatorIO io) {
    this.io = io;
    leadDisconnectedAlert = new Alert("Disconnected lead motor on elevator.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Disconnected follower motor on elevator.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(ioInputs);
    Logger.processInputs("Elevator", ioInputs);

    leadDisconnectedAlert.set(!ioInputs.leadSparkConnected);
    followerDisconnectedAlert.set(!ioInputs.followerSparkConnected);
  }
}
