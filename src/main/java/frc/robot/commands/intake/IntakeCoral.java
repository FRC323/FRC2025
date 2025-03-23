package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake.GroundIntakePosition;

public class IntakeCoral extends SequentialCommandGroup {
  public IntakeCoral(
      Elevator elevator, Arm arm, CoralIntake coralIntake, GroundIntake groundIntake) {
    addCommands(
        new ConditionalCommand(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL),
            new InstantCommand(),
            () -> elevator.getPosition() <= ElevatorPosition.TRAVEL.val),
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.HUMAN_PLAYER),
            new MoveArmToPosition(arm, Arm.ArmPosition.HUMAN_PLAYER),
            new MoveGroundIntakeToPosition(groundIntake, GroundIntakePosition.STOW),
            new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.IntakeOutput)));
  }
}
