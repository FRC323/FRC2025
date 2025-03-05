package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.coral.CoralIntake;

public class RunCoralOuttakeAuto extends Command {
  private CoralIntake intake;

  public RunCoralOuttakeAuto(CoralIntake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public void execute() {
    intake.outtake();
  }
}
