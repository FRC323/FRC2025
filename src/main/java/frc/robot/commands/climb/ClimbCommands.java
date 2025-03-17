package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimbCommands {
  public static Command manualClimberControl(Climber climber, DoubleSupplier percent) {
    return new ManualClimberControl(climber, percent);
  }

  public static Command RunClimberUp(Climber climber) {
    return new RunClimbUp(climber);
  }

  public static Command RunClimberDown(Climber climber) {
    return new RunClimbDown(climber);
  }

  public static Command ClimberStop(Climber climber) {
    return new ClimberStop(climber);
  }
}
