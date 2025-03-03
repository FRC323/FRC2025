package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class ElevatorSetOpsMode extends Command {
  private Elevator elevator;
  private ElevatorPosition position;

  public ElevatorSetOpsMode(Elevator elevator, ElevatorPosition position) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.position = position;
  }

  @Override
  public void initialize() {
    this.elevator.setOpMode(position);
  }

  @Override
  public void execute() {
    this.elevator.setOpMode(position);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
