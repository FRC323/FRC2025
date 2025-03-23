package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake.GroundIntakePosition;
import frc.robot.subsystems.intakes.ground.GroundIntakeConstants;

public class MoveToGroundPickup extends SequentialCommandGroup {
  public MoveToGroundPickup(
      Elevator elevator, Arm arm, GroundIntake groundIntake, CoralIntake coralIntake) {
    addCommands(
        // Step 1: Deploy ground intake
        new PrintCommand("Ground Pickup: Deploying ground intake"),
        new MoveGroundIntakeToPosition(groundIntake, GroundIntakePosition.DEPLOY),

        // Step 2: Wait for ground intake to finish deploying
        new WaitUntilCommand(() -> groundIntake.reachedDesiredPosition()),
        new PrintCommand("Ground Pickup: Ground intake deployed"),

        // Step 3: Move other components in parallel
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.Home),
            new MoveArmToPosition(arm, Arm.ArmPosition.GROUND_PICKUP),
            new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.IntakeOutput),
            new RunGroundIntake(groundIntake, () -> GroundIntakeConstants.IntakeOutput)));
  }
}
