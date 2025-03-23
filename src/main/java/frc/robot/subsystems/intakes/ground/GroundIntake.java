package frc.robot.subsystems.intakes.ground;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
  private final Alert wristDisconnectedAlert;
  private final Alert intakeDisconnectedAlert;

  private final GroundIntakeIO io;
  private final GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();

  public GroundIntake(GroundIntakeIO io) {
    this.io = io;
    wristDisconnectedAlert =
        new Alert("Disconnected wrist motor on Ground Intake", Alert.AlertType.kError);
    intakeDisconnectedAlert =
        new Alert("Disconnected intake motor on Ground Intake", Alert.AlertType.kError);
  }

  public enum GroundIntakePosition {
    STOW,
    DEPLOY,
    CLIMB
  }

  public enum GroundIntakeDirection {
    INTAKE,
    OUTTAKE,
    STOP
  }

  public void setWristPosition(GroundIntakePosition position) {
    switch (position) {
      case STOW:
        setWristPosition(GroundIntakeConstants.stowedPosition);
        break;
      case DEPLOY:
        setWristPosition(GroundIntakeConstants.deployedPosition);
        break;
      case CLIMB:
        setWristPosition(GroundIntakeConstants.climbPosition);
        break;
    }
  }

  public void setIntakeDirection(GroundIntakeDirection direction) {
    switch (direction) {
      case INTAKE:
        runIntakePercentOutput(GroundIntakeConstants.IntakeOutput);
        break;
      case OUTTAKE:
        runIntakePercentOutput(-GroundIntakeConstants.IntakeOutput);
        break;
      case STOP:
        runIntakePercentOutput(0);
        break;
    }
  }

  public boolean isStowed() {
    var currentWristPosition = io.getWristPosition();
    return MathUtil.isNear(GroundIntakeConstants.stowedPosition, currentWristPosition, 0.05);
  }

  public boolean isDeployed() {
    var currentWristPosition = io.getWristPosition();
    return MathUtil.isNear(GroundIntakeConstants.deployedPosition, currentWristPosition, 0.05);
  }

  public void runIntakePercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setIntakePercent(percent);
  }

  public void runWristPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setWristPercent(percent);
  }

  public void setWristPosition(double position) {
    position = MathUtil.clamp(position, 0, 1);
    io.setWristPosition(position);
  }

  public void stop() {
    runIntakePercentOutput(0);
    runWristPercentOutput(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GroundIntake", inputs);

    wristDisconnectedAlert.set(!inputs.wristSparkConnected);
    intakeDisconnectedAlert.set(!inputs.intakeSparkConnected);
  }

  public boolean reachedDesiredPosition() {
    return MathUtil.isNear(
        inputs.wristTargetPosition,
        inputs.wristCurrentPosition,
        GroundIntakeConstants.wristSetpointTolerance);
  }
}
