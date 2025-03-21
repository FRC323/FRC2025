package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final Alert leadDisconnectedAlert;

  public enum ClimberPosition {
    Stowed,
    Deploy,
    Climb
  }

  public Climber(ClimberIO io) {
    this.io = io;
    leadDisconnectedAlert =
        new Alert("Disconnected lead motor on climber.", Alert.AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
  }

  public double getTargetPosition() {
    return io.getTargetPosition();
  }

  public double getCurrentPosition() {
    return io.getCurrentPosition();
  }

  public void stop() {
    io.stop();
  }

  public void runPercentOutput(double percent, ClimberPosition position) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent, position);
  }

  public void runVoltageOutput(double voltage, ClimberPosition position) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    io.setVoltage(voltage, position);
  }

  public boolean reachedDesiredPosition() {
    return MathUtil.isNear(getTargetPosition(), getCurrentPosition(), 0.05);
  }
}
