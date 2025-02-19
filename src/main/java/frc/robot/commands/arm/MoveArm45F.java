package frc.robot.commands.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveArm45F extends Command {
  private Arm arm;

  public MoveArm45F(Arm arm) {
    addRequirements(arm);
    this.arm = arm;
  }

  @Override
  public void initialize() {
    this.arm.setAngleRadians(Units.degreesToRadians(45));
  }

  @Override
  public boolean isFinished() {
    return this.arm.reachedDesiredPosition();
  }
}
