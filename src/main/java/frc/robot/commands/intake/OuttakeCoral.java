package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;

public class OuttakeCoral extends SequentialCommandGroup {
  public OuttakeCoral(CoralIntake coralIntake) {
    addCommands(new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.IntakeOutput));
  }
}
