package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefLevel;

public class MoveElevatorToReefLevel extends Command {
  private Elevator elevator;
  private ReefLevel reefLevel;

  public MoveElevatorToReefLevel(Elevator elevator, ReefLevel reefLevel) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.reefLevel = reefLevel;
  }

  @Override
  public void initialize() {
    this.elevator.setHeight(this.reefLevel);
  }

  @Override
  public boolean isFinished() {
    return this.elevator.reachedDesiredPosition();
  }
}
