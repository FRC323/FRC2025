package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.coral.CoralIntake;

public class StopCoralIntake extends SequentialCommandGroup {
  public StopCoralIntake(CoralIntake coralIntake) {
    addCommands(new RunCoralIntake(coralIntake, () -> 0));
  }
}
