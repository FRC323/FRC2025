package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.commands.intake.MoveGroundIntakeToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake;

// put the robot in a travel pose
public class MoveToTravel extends SequentialCommandGroup {
  public MoveToTravel(
      Elevator elevator,
      Arm arm,
      CoralIntake coralIntake,
      AlgaeIntake algaeIntake,
      GroundIntake groundIntake) {

    addCommands(
        // Step 1: Deploy ground intake first
        new PrintCommand("MoveToTravel: Deploying ground intake"),
        new MoveGroundIntakeToPosition(groundIntake, GroundIntake.GroundIntakePosition.DEPLOY),

        // Step 2: Wait until ground intake completes deployment
        // new WaitUntilCommand(() ->
        // groundIntake.isAtTargetPosition(GroundIntake.GroundIntakePosition.DEPLOY)),

        // Step 3: Move arm and elevator to travel position
        new PrintCommand("MoveToTravel: Moving to travel position"),
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, ElevatorPosition.TRAVEL),
            new MoveArmToPosition(arm, ArmPosition.HOME)),
        // new RunCoralIntake(coralIntake, () -> 0.0),
        // new RunAlgaeIntake(algaeIntake, () -> 0.0)),
        new PrintCommand("elevator pos reached: " + elevator.reachedDesiredPosition()),
        new PrintCommand("arm pos reached: " + arm.reachedDesiredPosition()),

        // Step 4: Wait until arm and elevator are in position
        // new WaitUntilCommand(() ->
        //     elevator.reachedDesiredPosition() &&
        //     arm.isAtTargetPosition()),

        // Step 5: Stow the ground intake
        new PrintCommand("MoveToTravel: Stowing ground intake"),
        new MoveGroundIntakeToPosition(groundIntake, GroundIntake.GroundIntakePosition.STOW),

        // Step 6: Final completion message
        new PrintCommand("MoveToTravel: Complete"));
  }
}
