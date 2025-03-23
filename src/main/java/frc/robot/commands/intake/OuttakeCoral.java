package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntakeConstants;

public class OuttakeCoral extends SequentialCommandGroup {
  public OuttakeCoral(CoralIntake coralIntake, GroundIntake groundIntake) {
    addCommands(
        new ParallelCommandGroup(
            new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.IntakeOutput),
            new RunGroundIntake(groundIntake, () -> -GroundIntakeConstants.IntakeOutput)));
  }
}
