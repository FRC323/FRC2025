package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import org.littletonrobotics.junction.Logger;

public class MoveArmToL4Auto extends Command {
  private Arm arm;

  public MoveArmToL4Auto(Arm arm) {
    addRequirements(arm);
    this.arm = arm;
  }

  @Override
  public void initialize() {
    this.arm.setArmPosition(ArmPosition.REEF_LEVEL_4_CORAL);
    Logger.recordOutput("Auto/CommandRunning", "MoveArmToL4Auto.execute");
  }

  @Override
  public boolean isFinished() {
    boolean finished = this.arm.reachedDesiredPosition();
    Logger.recordOutput("Auto/CommandRunning", "MoveArmToL4Auto.isFinished: " + finished);
    return finished;
  }
}
