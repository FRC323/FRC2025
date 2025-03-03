package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  public static Command moveElevatorToPosition(Elevator elevator, ElevatorPosition position) {
    return new MoveElevatorToPosition(elevator, position);
  }

  public static Command manualElevatorControl(Elevator elevator, DoubleSupplier percentSupplier) {
    return new ManualElevatorControl(elevator, percentSupplier);
  }

  public static Command setOpMode(Elevator elevator, ElevatorPosition elevatorPosition) {
    return new ElevatorSetOpsMode(elevator, elevatorPosition);
  }
}
