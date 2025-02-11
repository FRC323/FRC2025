package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert leadDisconnectedAlert;

  public Arm(ArmIO io) {
    this.io = io;
    leadDisconnectedAlert = new Alert("Disconnected lead motor on arm.", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
  }

  public double getAngleRadians() {
    return inputs.currentPositionRadians;
  }

  public double getTargetAngleRadians() {
    return inputs.targetPositionRadians;
  }

  public void setAngleRadians(double targetAngleRadians) {
    io.setAngleRadians(targetAngleRadians);
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(inputs.targetPositionRadians - inputs.currentPositionRadians) <= 0.2;
  }
}
