package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimbCommands {
  public static Command manualClimberControl(Climber climber, DoubleSupplier percentSupplier) {
    return new ManualClimberControl(climber, percentSupplier);
  }
}
