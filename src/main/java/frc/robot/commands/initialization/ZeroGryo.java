package frc.robot.commands.initialization;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class ZeroGryo extends Command {
  private Drive drive;

  public ZeroGryo(Drive drive) {
    addRequirements(drive);
    this.drive = drive;
  }

  @Override
  public void execute() {
    this.drive.zeroGryo();
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
