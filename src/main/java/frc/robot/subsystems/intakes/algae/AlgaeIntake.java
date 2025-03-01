package frc.robot.subsystems.intakes.algae;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntake extends SubsystemBase {
  private final Alert spark1DisconnectedAlert;
  private final Alert spark2DisconnectedAlert;

  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIOInputsAutoLogged inputs = new AlgaeIntakeIOInputsAutoLogged();

  public AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
    spark1DisconnectedAlert =
        new Alert("Disconnected motor 1 on algae intake.", Alert.AlertType.kError);
    spark2DisconnectedAlert =
        new Alert("Disconnected motor 2 on algae intake.", Alert.AlertType.kError);
  }

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void intake() {
    runPercentOutput(1);
  }

  public void outtake() {
    runPercentOutput(-1);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeIntake", inputs);

    spark1DisconnectedAlert.set(!inputs.spark1SparkConnected);
    spark2DisconnectedAlert.set(!inputs.spark2SparkConnected);
  }
}
