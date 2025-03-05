package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class MoveElevatorToL4Auto extends Command {
  private Elevator elevator;

  public MoveElevatorToL4Auto(Elevator elevator) {
    addRequirements(elevator);
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    this.elevator.setPosition(ElevatorConstants.ReefLevel4CoralPosition);
    Logger.recordOutput("Auto/CommandRunning", "MoveElevatorToL4Auto.execute");
  }

  @Override
  public boolean isFinished() {
    boolean finished = this.elevator.reachedDesiredPosition();
    Logger.recordOutput("Auto/CommandRunning", "MoveElevatorToL4Auto.isFinished: " + finished);
    return finished;
  }
}
