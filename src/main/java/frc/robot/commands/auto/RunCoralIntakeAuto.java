package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.coral.CoralIntake;

public class RunCoralIntakeAuto extends Command {
  private CoralIntake intake;

  public RunCoralIntakeAuto(CoralIntake intake) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public void execute() {
    intake.intake();
  }

  @Override
  public boolean isFinished() {
    return intake.hasGamePiece();
  }
}
