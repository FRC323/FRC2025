package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class ScoreAlgae extends SequentialCommandGroup {
  public ScoreAlgae(
      Elevator elevator, ElevatorPosition elevatorPosition, Arm arm, ArmPosition armPosition) {
    addCommands(
        new ConditionalCommand(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL),
            new InstantCommand(),
            () -> elevator.getPosition() <= ElevatorPosition.TRAVEL.val),
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, elevatorPosition),
            new MoveArmToPosition(arm, armPosition)));
    // new RunAlgaeIntake(algaeIntake, () -> -AlgaeIntakeConstants.intakeOutput)));
  }
}
