package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;

public class AutoCommands {
  public static Command CoralIntakeAuto(CoralIntake coralIntake) {
    return new RunCoralIntakeAuto(coralIntake) /*.withTimeout(2)*/;
  }

  public static Command CoralOuttakeAuto(CoralIntake coralIntake) {
    return new RunCoralOuttakeAuto(coralIntake) /*.withTimeout(2)*/;
  }

  public static Command MoveElevatorToL4Auto(Elevator elevator) {
    return new MoveElevatorToL4Auto(elevator) /*.withTimeout(2)*/;
  }

  public static Command MoveArmToL4Auto(Elevator elevator) {
    return new MoveElevatorToL4Auto(elevator) /*.withTimeout(2)*/;
  }

  // public static Command MoveToScoreCoralAuto(
  //     Elevator elevator, ElevatorPosition elevatorPosition, Arm arm, ArmPosition armPosition) {
  //   return new MoveToScoreCoralAuto(
  //       elevator, elevatorPosition, arm, armPosition) /*.withTimeout(2)*/;
  // }

  // public static Command moveToTravelAuto(
  //     Elevator elevator, Arm arm, CoralIntake coralIntake, AlgaeIntake algaeIntake) {
  //   return new MoveToTravelAuto(elevator, arm, coralIntake, algaeIntake) /*.withTimeout(2)*/;
  // }
}
