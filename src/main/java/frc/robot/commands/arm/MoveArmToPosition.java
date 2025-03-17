package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import org.littletonrobotics.junction.Logger;

public class MoveArmToPosition extends Command {
  private Arm arm;
  private ArmPosition position;

  public MoveArmToPosition(Arm elevator, ArmPosition position) {
    addRequirements(elevator);
    this.arm = elevator;
    this.position = position;
  }

  @Override
  public void execute() {
    this.arm.setArmPosition(position);
    Logger.recordOutput("Arm/CommandRunning", true);
  }

  @Override
  public boolean isFinished() {
    boolean finished = this.arm.reachedDesiredPosition();
    Logger.recordOutput("Arm/CommandFinished", finished);
    return finished;
  }
}
