package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.MoveArmToPosition;
import frc.robot.commands.elevator.MoveElevatorToPosition;
import frc.robot.commands.intake.MoveGroundIntakeToPosition;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.ground.GroundIntake;

public class MoveToDeployClimber extends SequentialCommandGroup {
  public MoveToDeployClimber(
      Climber climber, Elevator elevator, Arm arm, GroundIntake groundIntake) {
    addCommands(
        new ParallelCommandGroup(
            new DeployClimber(climber),
            new MoveElevatorToPosition(elevator, ElevatorPosition.Home),
            new MoveArmToPosition(arm, ArmPosition.CLIMB_POSITION),
            new MoveGroundIntakeToPosition(
                groundIntake, GroundIntake.GroundIntakePosition.DEPLOY)));
  }
}
