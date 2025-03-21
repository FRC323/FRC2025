package frc.robot.commands.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberPosition;

public class DonkeyKong extends Command {
  private final Climber climber;
  private final double percent;

  public DonkeyKong(Climber climber, double percent) {
    this.climber = climber;
    this.percent = percent;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.runPercentOutput(percent, ClimberPosition.Climb);
  }

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(climber.getTargetPosition(), climber.getCurrentPosition(), 0.05);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      climber.stop();
    }
  }
}
