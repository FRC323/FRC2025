package frc.robot.commands.initialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;

public class OffsetCommands {
  public static Command storeDrivetrainOffsets(Drive drive) {
    return new StoreDrivetrainOffsets(drive);
  }

  public static Command storeArmOffsets(Arm arm) {
    return new StoreArmOffsets(arm);
  }
}
