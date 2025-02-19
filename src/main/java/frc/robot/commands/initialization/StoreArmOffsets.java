package frc.robot.commands.initialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class StoreArmOffsets extends Command {
  private Arm arm;

  public StoreArmOffsets(Arm arm) {
    addRequirements(arm);
    this.arm = arm;
  }

  @Override
  public void execute() {
    System.out.println("StoreArmOffsets.execute called");
    this.arm.storeOffset();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
