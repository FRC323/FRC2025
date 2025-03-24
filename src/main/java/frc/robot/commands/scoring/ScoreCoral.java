package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.commands.intake.MoveGroundIntakeToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake.GroundIntakePosition;

public class ScoreCoral extends SequentialCommandGroup {
  public ScoreCoral(
      Elevator elevator,
      ElevatorPosition elevatorPosition,
      Arm arm,
      ArmPosition armPosition,
      GroundIntake groundIntake) {
    addCommands(
        // Move elevator to travel position if needed
        new ConditionalCommand(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL),
            new InstantCommand(),
            () -> elevator.getPosition() <= ElevatorPosition.TRAVEL.val),

        // deploy ground intake if needed to get out of the way
        new ConditionalCommand(
            new SequentialCommandGroup(
                new MoveGroundIntakeToPosition(groundIntake, GroundIntakePosition.DEPLOY),
                new WaitUntilCommand(() -> groundIntake.reachedDesiredPosition())),
            new InstantCommand(),
            () ->
                (elevatorPosition == ElevatorPosition.REEF_LEVEL_1_CORAL
                        && armPosition == ArmPosition.REEF_LEVEL_1_CORAL)
                    || (elevatorPosition == ElevatorPosition.REEF_LEVEL_2_CORAL
                        && armPosition == ArmPosition.REEF_LEVEL_2_CORAL)),

        // Move elevator and arm to scoring position
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, elevatorPosition),
            new MoveArmToPosition(arm, armPosition)));
  }
}
