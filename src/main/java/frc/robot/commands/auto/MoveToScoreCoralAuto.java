// package frc.robot.commands.auto;

// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.arm.MoveArmToPosition;
// import frc.robot.commands.elevator.MoveElevatorToPosition;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.Arm.ArmPosition;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

// public class MoveToScoreCoralAuto extends SequentialCommandGroup {
//   public MoveToScoreCoralAuto(
//       Elevator elevator, ElevatorPosition elevatorPosition, Arm arm, ArmPosition armPosition) {
//     addCommands(
//         new ParallelCommandGroup(
//             new MoveElevatorToPosition(elevator, elevatorPosition),
//             new MoveArmToPosition(arm, armPosition)));
//   }
// }
