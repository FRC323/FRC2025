package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;

public class OuttakeCoralWSensor extends SequentialCommandGroup {
  public OuttakeCoralWSensor(CoralIntake coralIntake) {
    addCommands(
        new RepeatCommand(
            new ConditionalCommand(
                new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.IntakeOutput),
                new InstantCommand(),
                () -> coralIntake.isReefBranchDetected())));
  }
}
