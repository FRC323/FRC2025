package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class HPIntake extends SequentialCommandGroup {
  public HPIntake(Elevator elevator, Arm arm) {
    addCommands(
        Commands.parallel(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.HUMAN_PLAYER),
            new MoveArmToPosition(arm, Arm.ArmPosition.HUMAN_PLAYER)));
  }
}
