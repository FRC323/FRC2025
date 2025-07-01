package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.algae.AlgaeIntakeConstants;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake.GroundIntakePosition;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  public static Command StopGroundIntake(GroundIntake groundIntake) {
    return new RunGroundIntake(groundIntake, () -> 0.0);
  }

  public static Command MoveToGroundPickup(
      Elevator elevator, Arm arm, GroundIntake groundIntake, CoralIntake coralIntake) {
    return new MoveToGroundPickup(elevator, arm, groundIntake, coralIntake);
  }

  public static Command MoveToHumanPlayerPickup(
      Elevator elevator, Arm arm, GroundIntake groundIntake, CoralIntake coralIntake) {
    return new IntakeCoral(elevator, arm, coralIntake, groundIntake);
  }

  public static Command MoveGroundIntakeToPosition(
      GroundIntake groundIntake, GroundIntakePosition position) {
    return new MoveGroundIntakeToPosition(groundIntake, position);
  }

  // public static Command HoldCoralIntake(Arm arm, CoralIntake coralIntake) {
  //   return new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.normalOutput);
  // }
  public static Command HoldCoralAndDetectScore(
      Elevator elevator, Arm arm, CoralIntake coralIntake) {
    return new HoldCoralAndDetectScore(
        coralIntake, elevator, arm, () -> -CoralIntakeConstants.normalOutput);
  }

  public static Command HoldAlgaeIntake(Arm arm, Elevator elevator, AlgaeIntake algaeIntake) {
    return new RunAlgaeIntakeWithScaling(
        algaeIntake,
        arm,
        elevator,
        () -> AlgaeIntakeConstants.normalOutput,
        () -> AlgaeIntakeConstants.intakeOutput);
  }

  public static Command StopAlgaeIntake(AlgaeIntake algaeIntake) {
    return new StopAlgaeIntake(algaeIntake);
  }

  public static Command StopCoralIntake(CoralIntake coralIntake) {
    return new StopCoralIntake(coralIntake);
  }

  public static Command OuttakeAlgae(AlgaeIntake algaeIntake) {
    return new OuttakeAlgae(algaeIntake);
  }

  public static Command OuttakeCoral(CoralIntake coralIntake, GroundIntake groundIntake) {
    return new OuttakeCoral(coralIntake, groundIntake);
  }

  public static Command OuttakeCoralWSensor(
      CoralIntake coralIntake, GroundIntake groundIntake, Elevator elevator, Arm arm) {
    return new OuttakeCoralWSensor(coralIntake, groundIntake, elevator, arm);
  }

  public static Command ManualIntakeControl(
      CoralIntake coralIntake, AlgaeIntake algaeIntake, DoubleSupplier percentSupplier) {
    return new ManualIntakeControl(algaeIntake, coralIntake, percentSupplier);
  }

  public static Command ManualCoralIntakeControl(
      CoralIntake coralIntake, DoubleSupplier percentSupplier) {
    return new ManualCoralIntakeControl(coralIntake, percentSupplier);
  }

  public static Command ManualAlgaeIntakeControl(
      AlgaeIntake aglaeIntake, DoubleSupplier percentSupplier) {
    return new ManualAlgaeIntakeControl(aglaeIntake, percentSupplier);
  }

  // public static Command CoralIntake(Elevator elevator, Arm arm, CoralIntake coralIntake) {
  //   return new IntakeCoral(elevator, arm, coralIntake);
  // }

  public static Command RunCoralIntake(CoralIntake coralIntake) {
    return new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.normalOutput);
  }

  // TODO: RENAME AND REFACTOR
  public static Command RunCoralIntake2(CoralIntake coralIntake) {
    return new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.normalOutput);
  }

  public static Command AlgaeIntake(
      Elevator elevator,
      ElevatorPosition elevatorPosition,
      Arm arm,
      ArmPosition armPosition,
      AlgaeIntake algaeIntake) {
    return new IntakeAlgae(elevator, elevatorPosition, arm, armPosition, algaeIntake);
  }

  // public static Command IntakeAlgaeAndAlign(
  //     Drive drive,
  //     Vision vision,
  //     Elevator elevator,
  //     ElevatorPosition elevatorPosition,
  //     Arm arm,
  //     ArmPosition armPosition,
  //     AlgaeIntake algaeIntake) {
  //   return new SequentialCommandGroup(
  //       // Move elevator to travel position if needed
  //       new ConditionalCommand(
  //           new MoveElevatorToPosition(elevator, Elevator.ElevatorPosition.TRAVEL),
  //           new InstantCommand(),
  //           () -> elevator.getPosition() <= ElevatorPosition.TRAVEL.val),
  //       // Move elevator and arm to specified positions
  //       new ParallelCommandGroup(
  //           new MoveElevatorToPosition(elevator, elevatorPosition),
  //           new MoveArmToPosition(arm, armPosition)),
  //       // Align to reef and run algae intake in parallel
  //       new ParallelCommandGroup(
  //           AutoCommands.AlignToReefBestTag(drive, vision, PoleSide.LEFT),
  //           new RunAlgaeIntake(algaeIntake, () -> AlgaeIntakeConstants.intakeOutput)
  //               .withTimeout(1.5)));
  // }
}
