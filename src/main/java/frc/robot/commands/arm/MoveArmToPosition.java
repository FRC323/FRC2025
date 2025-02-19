package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;

public class MoveArmToPosition extends Command {
  private Arm arm;
  private ArmPosition position;

  public MoveArmToPosition(Arm elevator, ArmPosition position) {
    addRequirements(elevator);
    this.arm = elevator;
    this.position = position;
  }

  @Override
  public void initialize() {
    this.arm.setArmPosition(position);
  }

  @Override
  public boolean isFinished() {
    return this.arm.reachedDesiredPosition();
  }
}
