package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public enum ClimberState {
    DEPLOYED,
    CLIMBED,
    STOWED
  }

  private final Alert leadDisconnectedAlert;

  public Climber(ClimberIO io) {
    this.io = io;
    leadDisconnectedAlert =
        new Alert("Disconnected lead motor on climber.", Alert.AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
  }

  private void deploy() {
    inputs.currentState = ClimberState.DEPLOYED;
  }

  private void climb() {
    inputs.currentState = ClimberState.CLIMBED;
  }

  public void stow() {
    inputs.currentState = ClimberState.STOWED;
  }
}
