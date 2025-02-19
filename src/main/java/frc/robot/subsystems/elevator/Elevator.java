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

  public Elevator(ElevatorIO io) {
    this.io = io;
    leadDisconnectedAlert = new Alert("Disconnected lead motor on elevator.", AlertType.kError);
    followerDisconnectedAlert =
        new Alert("Disconnected follower motor on elevator.", AlertType.kError);
  }

  public enum ElevatorPosition {
    Home(0.0),
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

  public double getHeightInches() {
    return inputs.currentHeightPosition;
  }

  public void setHeight(double targetHeightPosition) {
    var position =
        MathUtil.clamp(
            targetHeightPosition,
            ElevatorConstants.minElevatorHeight,
            ElevatorConstants.maxElevatorHeight);

    io.setTargetHeight(position);
  }

  public void setHeight(ElevatorPosition position) {
    switch (position) {
      case Home:
        setHeight(ElevatorConstants.HomePosition);
        break;
      case REEF_LEVEL_1_CORAL:
        setHeight(ElevatorConstants.ReefLevel1CoralPosition);
        break;
      case REEF_LEVEL_2_CORAL:
        setHeight(ElevatorConstants.ReefLevel2CoralPosition);
        break;
      case REEF_LEVEL_3_CORAL:
        setHeight(ElevatorConstants.ReefLevel3CoralPosition);
        break;
      case REEF_LEVEL_4_CORAL:
        setHeight(ElevatorConstants.ReefLevel4CoralPosition);
        break;
      case HUMAN_PLAYER:
        setHeight(ElevatorConstants.HumanPlayerPosition);
        break;
      case REEF_LEVEL_1_ALGAE:
        setHeight(ElevatorConstants.ReefLevel1AlgaePosition);
        break;
      case REEF_LEVEL_2_ALGAE:
        setHeight(ElevatorConstants.ReefLevel2AlgaePosition);
        break;
      case ALGAE_BARGE:
        setHeight(ElevatorConstants.BargePosition);
        break;
      case ALGAE_PROCESSOR:
        setHeight(ElevatorConstants.ProcessorPosition);
        break;
    }
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(inputs.targetHeightPosition - inputs.currentHeightPosition) < 0.1;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
    followerDisconnectedAlert.set(!inputs.followerSparkConnected);
  }
}
