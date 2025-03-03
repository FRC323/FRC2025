package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final Alert leadDisconnectedAlert;

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

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void runVoltageOutput(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    io.setVoltage(voltage);
  }
}
