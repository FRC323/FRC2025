package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.commands.intake.RunAlgaeIntake;
import frc.robot.commands.intake.RunCoralIntake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;

// put the robot in a travel pose

public class MoveToTravel extends SequentialCommandGroup {
  public MoveToTravel(
      Elevator elevator, Arm arm, CoralIntake coralIntake, AlgaeIntake algaeIntake) {
    addCommands(
        new ParallelCommandGroup(
            new MoveElevatorToPosition(elevator, ElevatorPosition.TRAVEL),
            new MoveArmToPosition(arm, ArmPosition.HOME),
            new RunCoralIntake(coralIntake, () -> 0.0),
            new RunAlgaeIntake(algaeIntake, () -> 0.0)));
  }
}
