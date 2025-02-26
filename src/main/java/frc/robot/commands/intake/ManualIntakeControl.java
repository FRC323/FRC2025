package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import java.util.function.DoubleSupplier;

public class ManualIntakeControl extends SequentialCommandGroup {

  public ManualIntakeControl(
      AlgaeIntake algaeIntake, CoralIntake coralIntake, DoubleSupplier percentSupplier) {
    addRequirements(coralIntake);
    addRequirements(algaeIntake);

    addCommands(
        new ParallelCommandGroup(
            new ManualAlgaeIntakeControl(algaeIntake, percentSupplier),
            new ManualCoralIntakeControl(coralIntake, percentSupplier)));
  }
}
