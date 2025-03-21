package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.Climber;

public class MoveToDonkeyKong extends SequentialCommandGroup {
  public MoveToDonkeyKong(Climber climber) {
    addCommands(new RepeatCommand(new DonkeyKong(climber, .8)));
  }
}
