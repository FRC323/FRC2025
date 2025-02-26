package frc.robot.subsystems.intakes.coral;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants;

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

    inputs.intakeMode = IntakeMode.None;
  }

  public enum IntakeMode {
    None,
    Intake,
    Outtake
  }

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void intake() {
    inputs.intakeMode = IntakeMode.Intake;
    runPercentOutput(1);
  }

  public void outtake() {
    inputs.intakeMode = IntakeMode.Outtake;
    runPercentOutput(-1);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralIntake", inputs);

    spark1DisconnectedAlert.set(!inputs.spark1SparkConnected);
    spark2DisconnectedAlert.set(!inputs.spark2SparkConnected);

    //TODO: cancel captured current when spitting the game piece out
    if (inputs.totalOutputCurrent >= CoralIntakeConstants.capturingPieceCurrent) {
      io.setPercent(CoralIntakeConstants.capturedCurrentOutput);
    }
    
  }
}
