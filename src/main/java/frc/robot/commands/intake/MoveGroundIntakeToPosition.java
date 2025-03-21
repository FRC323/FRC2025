package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake.GroundIntakePosition;

public class MoveGroundIntakeToPosition extends Command {
  private GroundIntake groundIntake;
  private GroundIntakePosition position;

  public MoveGroundIntakeToPosition(GroundIntake groundIntake, GroundIntakePosition position) {
    addRequirements(groundIntake);
    this.groundIntake = groundIntake;
    this.position = position;
  }

  @Override
  public void initialize() {
    this.groundIntake.setWristPosition(position);
  }

  @Override
  public boolean isFinished() {
    boolean finished = this.groundIntake.reachedDesiredPosition();
    if (finished && this.position == GroundIntakePosition.STOW) {
      this.groundIntake.runIntakePercentOutput(0);
    }
    return finished;
  }
}
