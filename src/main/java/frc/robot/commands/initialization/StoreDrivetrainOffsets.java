package frc.robot.commands.initialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class StoreDrivetrainOffsets extends Command {
  private Drive drive;

  public StoreDrivetrainOffsets(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
  }

  @Override
  public void execute() {
    this.drive.setDrivetrainOffsets();
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
