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
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  public static Command HoldCoralIntake(CoralIntake coralIntake) {
    return new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.normalOutput);
  }

  public static Command HoldAlgaeIntake(AlgaeIntake algaeIntake) {
    return new RunAlgaeIntake(algaeIntake, () -> AlgaeIntakeConstants.normalOutput);
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

  public static Command OuttakeCoral(CoralIntake coralIntake) {
    return new OuttakeCoral(coralIntake);
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

  public static Command CoralIntake(Elevator elevator, Arm arm, CoralIntake coralIntake) {
    return new IntakeCoral(elevator, arm, coralIntake);
  }

  public static Command RunCoralIntake(CoralIntake coralIntake) {
    return new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.normalOutput);
  }

  public static Command AlgaeIntake(
      Elevator elevator,
      ElevatorPosition elevatorPosition,
      Arm arm,
      ArmPosition armPosition,
      AlgaeIntake algaeIntake) {
    return new IntakeAlgae(elevator, elevatorPosition, arm, armPosition, algaeIntake);
  }
}
