package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final Alert leadDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double targetPosition = 0.0;

  public Elevator(ElevatorIO io) {
    this.io = io;
    leadDisconnectedAlert = new Alert("Disconnected lead motor on elevator.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Disconnected follower motor on elevator.", AlertType.kError);
  }

  public enum ElevatorPosition {
    Home(0.0),
    TRAVEL(ElevatorConstants.TravelPosition),
    REEF_LEVEL_1_CORAL(ElevatorConstants.ReefLevel1CoralPosition),
    REEF_LEVEL_2_CORAL(ElevatorConstants.ReefLevel2CoralPosition),
    REEF_LEVEL_3_CORAL(ElevatorConstants.ReefLevel3CoralPosition),
    REEF_LEVEL_4_CORAL(ElevatorConstants.ReefLevel4CoralPosition),
    HUMAN_PLAYER(ElevatorConstants.HumanPlayerPosition),
    REEF_LEVEL_1_ALGAE(ElevatorConstants.ReefLevel1AlgaePosition),
    REEF_LEVEL_2_ALGAE(ElevatorConstants.ReefLevel2AlgaePosition),
    ALGAE_BARGE(ElevatorConstants.BargePosition),
    ALGAE_PROCESSOR(ElevatorConstants.ProcessorPosition);

    public final double val;

    ElevatorPosition(double val) {
      this.val = val;
    }
  }

  public boolean elevatorInGroundIntakePosition() {
    return MathUtil.isNear(ElevatorConstants.HomePosition, inputs.leadEncoderPosition, .1);
  }

  public void stop() {
    this.targetPosition = 0.0;
    io.setPercent(0);
  }

  public boolean isInReefScoringPosition() {
    return MathUtil.isNear(
            ElevatorConstants.ReefLevel1CoralPosition, inputs.leadEncoderPosition, 0.5)
        || MathUtil.isNear(
            ElevatorConstants.ReefLevel2CoralPosition, inputs.leadEncoderPosition, 0.5)
        || MathUtil.isNear(
            ElevatorConstants.ReefLevel3CoralPosition, inputs.leadEncoderPosition, 0.5)
        || MathUtil.isNear(
            ElevatorConstants.ReefLevel4CoralPosition, inputs.leadEncoderPosition, 0.5);
  }

  public boolean isAtBottom() {
    return inputs.isAtBottom;
  }

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void runVoltageOutput(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    io.setVoltage(voltage);
  }

  public double getPosition() {
    return inputs.leadEncoderPosition;
  }

  public void setPosition(double targetPosition) {
    var target =
        MathUtil.clamp(
            targetPosition,
            ElevatorConstants.minElevatorHeight,
            ElevatorConstants.maxElevatorHeight);

    this.targetPosition = target;
    Logger.recordOutput("Elevator/TargetPosition", target);
    io.setPosition(target);
  }

  public void setPosition(ElevatorPosition position) {
    switch (position) {
      case Home:
        setPosition(ElevatorConstants.HomePosition);
        break;
      case TRAVEL:
        setPosition(ElevatorConstants.TravelPosition);
        break;
      case REEF_LEVEL_1_CORAL:
        setPosition(ElevatorConstants.ReefLevel1CoralPosition);
        break;
      case REEF_LEVEL_2_CORAL:
        setPosition(ElevatorConstants.ReefLevel2CoralPosition);
        break;
      case REEF_LEVEL_3_CORAL:
        setPosition(ElevatorConstants.ReefLevel3CoralPosition);
        break;
      case REEF_LEVEL_4_CORAL:
        setPosition(ElevatorConstants.ReefLevel4CoralPosition);
        break;
      case HUMAN_PLAYER:
        setPosition(ElevatorConstants.HumanPlayerPosition);
        break;
      case REEF_LEVEL_1_ALGAE:
        setPosition(ElevatorConstants.ReefLevel1AlgaePosition);
        break;
      case REEF_LEVEL_2_ALGAE:
        setPosition(ElevatorConstants.ReefLevel2AlgaePosition);
        break;
      case ALGAE_BARGE:
        setPosition(ElevatorConstants.BargePosition);
        break;
      case ALGAE_PROCESSOR:
        setPosition(ElevatorConstants.ProcessorPosition);
        break;
    }
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(this.targetPosition - inputs.leadEncoderPosition) < 0.5;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
    followerDisconnectedAlert.set(!inputs.followerSparkConnected);
  }

  public boolean isGoingDown(ElevatorPosition newPosition) {
    double currentPosition = inputs.leadEncoderPosition;
    return newPosition.val < currentPosition;
  }
}
