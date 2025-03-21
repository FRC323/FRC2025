package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class MoveToStowClimber extends SequentialCommandGroup {
  public MoveToStowClimber(Climber climber) {
    addCommands(new RepeatCommand(new StowClimber(climber, 1)));
  }
}
