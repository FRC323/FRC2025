package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.RunCoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;

public class ScoreCoralAuto extends SequentialCommandGroup {
  public ScoreCoralAuto(CoralIntake coralIntake) {
    addCommands(
        new ConditionalCommand(
                new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.IntakeOutput),
                new InstantCommand(),
                () -> coralIntake.isReefBranchDetected())
            .withTimeout(5));
  }
}
