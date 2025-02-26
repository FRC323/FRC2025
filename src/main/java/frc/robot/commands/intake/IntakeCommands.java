package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import java.util.function.DoubleSupplier;

public class IntakeCommands {
  public static Command HPIntake(Elevator elevator, Arm arm) {
    return new HPIntake(elevator, arm);
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
}
