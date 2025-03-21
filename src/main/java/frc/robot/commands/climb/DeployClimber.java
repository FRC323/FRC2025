package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberPosition;

public class DeployClimber extends Command {
  private final Climber climber;
  private final double percent;

  public DeployClimber(Climber climber, double percent) {
    this.climber = climber;
    this.percent = percent;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.runPercentOutput(percent, ClimberPosition.Deploy);
  }

  @Override
  public boolean isFinished() {
    return climber.reachedDesiredPosition();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      // If interrupted, stop the climber
      climber.stop();
    }
  }
}
