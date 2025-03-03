package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class ScoreCoralOp extends SequentialCommandGroup {
  public ScoreCoralOp(Elevator elevator, Arm arm) {
    addCommands(
        new PrintCommand("MOving to travel"),
        new ConditionalCommand(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL),
            new InstantCommand(),
            () -> elevator.getPosition() <= ElevatorPosition.TRAVEL.val),
        new PrintCommand("MOved to Travel"),
        new ParallelCommandGroup(
            new PrintCommand("Moving Elevator To Ops Command: " + elevator.getOpsModePosition()),
            new MoveElevatorToPosition(elevator, elevator.getOpsModePosition()),
            new PrintCommand(
                "Moving Arm To Ops Command: "
                    + getArmPositionFromElevator(elevator.getOpsModePosition())),
            new MoveArmToPosition(arm, getArmPositionFromElevator(elevator.getOpsModePosition()))));
  }

  private ArmPosition getArmPositionFromElevator(ElevatorPosition elevatorPosition) {
    if (elevatorPosition == ElevatorPosition.REEF_LEVEL_1_CORAL)
      return ArmPosition.REEF_LEVEL_1_CORAL;
    if (elevatorPosition == ElevatorPosition.REEF_LEVEL_2_CORAL)
      return ArmPosition.REEF_LEVEL_2_CORAL;
    if (elevatorPosition == ElevatorPosition.REEF_LEVEL_3_CORAL)
      return ArmPosition.REEF_LEVEL_3_CORAL;
    if (elevatorPosition == ElevatorPosition.REEF_LEVEL_4_CORAL)
      return ArmPosition.REEF_LEVEL_4_CORAL;
    return ArmPosition.HOME;
  }
}
