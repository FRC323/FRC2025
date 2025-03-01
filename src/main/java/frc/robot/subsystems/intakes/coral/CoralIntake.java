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

  public enum IntakeState {
    IDLE,
    INTAKING,
    HOLDING_PIECE,
    OUTTAKING
  }

  private final Timer currentSpikeTimer = new Timer();

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
    inputs.state = IntakeState.INTAKING;
    runPercentOutput(CoralIntakeConstants.normalOutput);
  }

  public void outtake() {
    inputs.state = IntakeState.OUTTAKING;
    runPercentOutput(-CoralIntakeConstants.normalOutput);
  }

  public void stop() {
    inputs.state = IntakeState.IDLE;
    runPercentOutput(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);

    spark1DisconnectedAlert.set(!inputs.spark1SparkConnected);
    spark2DisconnectedAlert.set(!inputs.spark2SparkConnected);

    // switch (inputs.state) {
    //   case IDLE:
    //     currentSpikeTimer.stop();
    //     runPercentOutput(0);
    //     break;

    //   case INTAKING:
    //     if (inputs.totalOutputCurrent >= CoralIntakeConstants.capturingPieceCurrent) {
    //       if (!currentSpikeTimer.isRunning()) {
    //         currentSpikeTimer.restart();
    //       }
    //       if (currentSpikeTimer.hasElapsed(CoralIntakeConstants.currentSpikeDuration)) {
    //         inputs.state = IntakeState.HOLDING_PIECE;
    //       }
    //     } else {
    //       currentSpikeTimer.stop();
    //     }
    //     runPercentOutput(CoralIntakeConstants.normalOutput);
    //     break;

    //   case HOLDING_PIECE:
    //     runPercentOutput(CoralIntakeConstants.capturedCurrentOutput);
    //     break;

    //   case OUTTAKING:
    //     runPercentOutput(-CoralIntakeConstants.normalOutput);
    //     break;
    // }

    Logger.recordOutput("CoralIntake/State", inputs.state.toString());
    Logger.recordOutput("CoralIntake/CurrentTimer", currentSpikeTimer.get());
  }
}
