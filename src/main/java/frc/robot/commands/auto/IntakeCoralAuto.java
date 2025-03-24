package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.commands.intake.MoveGroundIntakeToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake.GroundIntakePosition;

public class IntakeCoralAuto extends SequentialCommandGroup {
  public IntakeCoralAuto(
      Elevator elevator, Arm arm, CoralIntake coralIntake, GroundIntake groundIntake) {
    addCommands(
        // Step 1: Deploy ground intake
        new PrintCommand("Coral Intake: Deploying ground intake"),
        new MoveGroundIntakeToPosition(groundIntake, GroundIntakePosition.DEPLOY),
        new WaitUntilCommand(() -> groundIntake.reachedDesiredPosition()),
        new PrintCommand("Coral Intake: Ground intake deployed"),

        // Step 2: Move elevator to TRAVEL position if needed
        new ConditionalCommand(
            new SequentialCommandGroup(
                new PrintCommand("Coral Intake: Moving elevator to travel position first"),
                new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL),
                new WaitUntilCommand(() -> elevator.reachedDesiredPosition()),
                new PrintCommand("Coral Intake: Elevator at travel position")),
            new InstantCommand(),
            () -> elevator.getPosition() < ElevatorPosition.TRAVEL.val),

        // Step 3: Move to human player position
        new PrintCommand("Coral Intake: Moving to human player position"),
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.HUMAN_PLAYER),
            new MoveArmToPosition(arm, Arm.ArmPosition.HUMAN_PLAYER)),

        // Step 4: Wait until arm and elevator are in position
        new WaitUntilCommand(
            () -> elevator.reachedDesiredPosition() && arm.reachedDesiredPosition()),
        new PrintCommand("Coral Intake: At human player position, intaking"),

        // Step 5: Stow ground intake
        new PrintCommand("Coral Intake: Stowing ground intake"),
        new MoveGroundIntakeToPosition(groundIntake, GroundIntakePosition.STOW),
        new WaitUntilCommand(() -> groundIntake.reachedDesiredPosition()));
    // new PrintCommand("Coral Intake: Ground intake stowed"),
    // new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.IntakeOutput));
  }
}
