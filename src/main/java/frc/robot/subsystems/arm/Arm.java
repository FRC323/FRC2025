package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public enum ArmPosition {
    HOME(ArmConstants.HomePosition),
    REEF_LEVEL_1_CORAL(ArmConstants.ReefLevel1CoralPosition),
    REEF_LEVEL_2_CORAL(ArmConstants.ReefLevel2CoralPosition),
    REEF_LEVEL_3_CORAL(ArmConstants.ReefLevel3CoralPosition),
    REEF_LEVEL_4_CORAL(ArmConstants.ReefLevel4CoralPosition),
    HUMAN_PLAYER(ArmConstants.HumanPlayerPosition),
    REEF_LEVEL_1_ALGAE(ArmConstants.ReefLevel1AlgaePosition),
    REEF_LEVEL_2_ALGAE(ArmConstants.ReefLevel2AlgaePosition),
    ALGAE_BARGE(ArmConstants.BargePosition),
    ALGAE_PROCESSOR(ArmConstants.ProcessorPosition),
    GROUND_PICKUP(ArmConstants.GroundPickupPosition),
    CLIMB_POSITION(ArmConstants.ClimbPosition);

    public final double val;

    ArmPosition(double val) {
      this.val = val;
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    leadDisconnectedAlert.set(!inputs.leadSparkConnected);
    SmartDashboard.putNumber("Arm.Position", inputs.currentAbsolutePosition);
  }

  public double getAngleRadians() {
    return inputs.currentAbsolutePosition;
  }

  public double getTargetAngleRadians() {
    return inputs.targetPosition;
  }

  public boolean armInGroundIntakePosition() {
    return MathUtil.isNear(ArmConstants.GroundPickupPosition, inputs.currentAbsolutePosition, .1);
  }

  // public boolean isArmMoving() {
  //   return MathUtil.isNear(
  //       inputs.currentAbsolutePosition, inputs.targetPosition, ArmConstants.SetpointTolerance);
  // }

  public void setArmPosition(ArmPosition position) {
    switch (position) {
      case HOME:
        setAngleRadians(ArmConstants.HomePosition);
        break;
      case REEF_LEVEL_1_CORAL:
        setAngleRadians(ArmConstants.ReefLevel1CoralPosition);
        break;
      case REEF_LEVEL_2_CORAL:
        setAngleRadians(ArmConstants.ReefLevel2CoralPosition);
        break;
      case REEF_LEVEL_3_CORAL:
        setAngleRadians(ArmConstants.ReefLevel3CoralPosition);
        break;
      case REEF_LEVEL_4_CORAL:
        setAngleRadians(ArmConstants.ReefLevel4CoralPosition);
        break;
      case HUMAN_PLAYER:
        setAngleRadians(ArmConstants.HumanPlayerPosition);
        break;
      case REEF_LEVEL_1_ALGAE:
        setAngleRadians(ArmConstants.ReefLevel1AlgaePosition);
        break;
      case REEF_LEVEL_2_ALGAE:
        setAngleRadians(ArmConstants.ReefLevel2AlgaePosition);
        break;
      case ALGAE_BARGE:
        setAngleRadians(ArmConstants.BargePosition);
        break;
      case ALGAE_PROCESSOR:
        setAngleRadians(ArmConstants.ProcessorPosition);
        break;
      case GROUND_PICKUP:
        setAngleRadians(ArmConstants.GroundPickupPosition);
        break;
      case CLIMB_POSITION:
        setAngleRadians(ArmConstants.ClimbPosition);
        break;
    }
  }

  public void stop() {
    io.setVoltage(0);
  }

  public boolean isInInvertedRange() {
    double position = inputs.currentAbsolutePosition;
    return position >= ArmConstants.invertedStartPosition
        && position <= ArmConstants.invertedEndPosition;
  }

  public void runPercentOutput(double percent) {
    percent = MathUtil.clamp(percent, -1, 1);
    io.setPercent(percent);
  }

  public void setAngleRadians(double targetAngleRadians) {
    io.setAngleRadians(targetAngleRadians);
  }

  public void setAngleDegrees(double targetAngleDegrees) {
    this.setAngleRadians(Units.degreesToRadians(targetAngleDegrees));
  }

  public boolean reachedDesiredPosition() {
    return MathUtil.isNear(
        inputs.targetPosition, inputs.currentAbsolutePosition, ArmConstants.SetpointTolerance);
  }

  public boolean isInReefScoringPosition() {
    return MathUtil.isNear(
            ArmConstants.ReefLevel1CoralPosition, inputs.currentAbsolutePosition, 0.1)
        || MathUtil.isNear(
            ArmConstants.ReefLevel2CoralPosition, inputs.currentAbsolutePosition, 0.1)
        || MathUtil.isNear(
            ArmConstants.ReefLevel3CoralPosition, inputs.currentAbsolutePosition, 0.1)
        || MathUtil.isNear(
            ArmConstants.ReefLevel4CoralPosition, inputs.currentAbsolutePosition, 0.1);
  }
}
