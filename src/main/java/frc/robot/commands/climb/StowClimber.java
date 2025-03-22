package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberPosition;

public class StowClimber extends Command {
  private final Climber climber;
  // private double percent;

  public StowClimber(Climber climber) {
    this.climber = climber;
    // this.percent = percent;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    // climber.runPercentOutput(percent, ClimberPosition.Stowed);
    climber.setPosition(ClimberPosition.Stowed);
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
