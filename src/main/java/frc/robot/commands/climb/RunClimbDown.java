package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.*;

public class RunClimbDown extends Command {
  private Climber climber;

  public RunClimbDown(Climber climber) {
    addRequirements(climber);
    this.climber = climber;
  }

  @Override
  public void initialize() {
    this.climber.runPercentOutput(-.8);
  }
}
