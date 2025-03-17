package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveArmToHome extends Command {
  private Arm arm;

  public MoveArmToHome(Arm arm) {
    addRequirements(arm);
    this.arm = arm;
  }

  @Override
  public void initialize() {
    this.arm.setAngleDegrees(90);
  }

  @Override
  public boolean isFinished() {
    return this.arm.reachedDesiredPosition();
  }
}
