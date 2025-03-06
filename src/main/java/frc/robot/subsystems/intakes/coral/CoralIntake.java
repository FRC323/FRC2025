package frc.robot.subsystems.intakes.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralIntake extends SubsystemBase {
  private final Alert spark1DisconnectedAlert;
  private final Alert spark2DisconnectedAlert;

  private final CoralIntakeIO io;
  private final CoralIntakeIOInputsAutoLogged inputs = new CoralIntakeIOInputsAutoLogged();

  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
    spark1DisconnectedAlert =
        new Alert("Disconnected motor 1 on coral intake.", Alert.AlertType.kError);
    spark2DisconnectedAlert =
        new Alert("Disconnected motor 2 on coral intake.", Alert.AlertType.kError);
  }

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void intake() {
    runPercentOutput(CoralIntakeConstants.normalOutput);
  }

  public void outtake() {
    runPercentOutput(-CoralIntakeConstants.IntakeOutput);
  }

  public void stop() {
    runPercentOutput(0);
  }

  public boolean hasGamePiece() {
    return inputs.hasGamePiece;
  }

  private final Timer currentSpikeTimer = new Timer();

  private boolean detectGamePieceIntake() {
    double totalCurrent = inputs.totalOutputCurrent;

    if (totalCurrent >= CoralIntakeConstants.capturedCurrentOutput) {
      if (!currentSpikeTimer.isRunning()) {
        currentSpikeTimer.restart();
      }
      if (currentSpikeTimer.hasElapsed(0.25)) {
        return true;
      }
    } else {
      currentSpikeTimer.stop();
    }
    return false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);

    spark1DisconnectedAlert.set(!inputs.spark1SparkConnected);
    spark2DisconnectedAlert.set(!inputs.spark2SparkConnected);

    if (inputs.spark1SpeedPercent > 0) { // positive, so intaking
      inputs.hasGamePiece = detectGamePieceIntake();
      // else {
      // inputs.hasGamePiece = detectGamePieceOuttake();
    }

    Logger.recordOutput("CoralIntake/CurrentTimer", currentSpikeTimer.get());
  }
}
