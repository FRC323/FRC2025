package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;

public class StopAlgaeIntake extends SequentialCommandGroup {
  public StopAlgaeIntake(AlgaeIntake algaeIntake) {
    addCommands(new RunAlgaeIntake(algaeIntake, () -> 0));
  }
}
