package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreCoralL2 extends SequentialCommandGroup {
  public ScoreCoralL2(Elevator elevator, Arm arm) {
    addCommands(
        Commands.parallel(
            new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.REEF_LEVEL_2_CORAL),
            new MoveArmToPosition(arm, Arm.ArmPosition.REEF_LEVEL_2_CORAL)));
  }
}
