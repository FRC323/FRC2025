package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberPosition;

public class DeployClimber extends Command {
  private final Climber climber;

  public DeployClimber(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setPosition(ClimberPosition.Deploy);
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
