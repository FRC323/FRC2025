package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(Elevator elevator, Arm arm, CoralIntake coralIntake) {
    addCommands(
        new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL).withTimeout(10),
        new ParallelCommandGroup(
                new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.HUMAN_PLAYER),
                new MoveArmToPosition(arm, Arm.ArmPosition.HUMAN_PLAYER),
                new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.IntakeOutput))
            .withTimeout(10));
  }
}
