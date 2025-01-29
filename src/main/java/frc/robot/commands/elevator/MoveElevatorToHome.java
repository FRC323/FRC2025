package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToHome extends Command {
  private Elevator elevator;

  public MoveElevatorToHome(Elevator elevator) {
    addRequirements(elevator);
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    this.elevator.setHeight(0);
  }

  @Override
  public boolean isFinished() {
    return this.elevator.reachedDesiredPosition();
  }
}
