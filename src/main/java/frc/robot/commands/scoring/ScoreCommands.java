package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.ground.GroundIntake;

public class ScoreCommands {
  public static Command ScoreCoral(
      Elevator elevator,
      ElevatorPosition elevatorPosition,
      Arm arm,
      ArmPosition armPosition,
      GroundIntake groundIntake) {
    return new ScoreCoral(elevator, elevatorPosition, arm, armPosition, groundIntake);
  }

  public static Command ScoreCoral(Elevator elevator, Arm arm) {
    return new ScoreCoralOp(elevator, arm);
  }

  public static Command ScoreAlgae(
      Elevator elevator, ElevatorPosition elevatorPosition, Arm arm, ArmPosition armPosition) {
    return new ScoreAlgae(elevator, elevatorPosition, arm, armPosition);
  }
}
