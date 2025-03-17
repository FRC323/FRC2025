package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.algae.AlgaeIntakeConstants;

public class OuttakeAlgae extends SequentialCommandGroup {
  public OuttakeAlgae(AlgaeIntake algaeIntake) {
    addCommands(new RunAlgaeIntake(algaeIntake, () -> -AlgaeIntakeConstants.intakeOutput));
  }
}
