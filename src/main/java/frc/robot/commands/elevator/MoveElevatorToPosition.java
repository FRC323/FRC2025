package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MoveElevatorToPosition extends Command {
  private Elevator elevator;
  private ElevatorPosition position;

  public MoveElevatorToPosition(Elevator elevator, ElevatorPosition position) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.position = position;
  }

  @Override
  public void initialize() {
    this.elevator.setHeight(position);
  }

  @Override
  public boolean isFinished() {
    return this.elevator.reachedDesiredPosition();
  }
}
