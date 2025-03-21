package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        new MoveGroundIntakeToPosition(groundIntake, GroundIntakePosition.DEPLOY),
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.Home),
            new MoveArmToPosition(arm, Arm.ArmPosition.GROUND_PICKUP),
            new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.IntakeOutput),
            new RunGroundIntake(groundIntake, () -> GroundIntakeConstants.IntakeOutput)));
  }
}
