package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class AlgaeL2Intake extends SequentialCommandGroup {
  public AlgaeL2Intake(Elevator elevator, Arm arm) {
    addCommands(
        Commands.parallel(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.REEF_LEVEL_2_ALGAE),
            new MoveArmToPosition(arm, Arm.ArmPosition.REEF_LEVEL_2_ALGAE)));
  }
}
