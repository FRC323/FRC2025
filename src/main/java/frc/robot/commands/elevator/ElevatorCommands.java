package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ReefLevel;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  public static Command moveElevatorToHome(Elevator elevator) {
    return new MoveElevatorToHome(elevator);
  }

  public static Command MoveElevatorToReefLevel(Elevator elevator, ReefLevel reefLevel) {
    return new MoveElevatorToReefLevel(elevator, reefLevel);
  }

  public static Command ManualElevatorControl(Elevator elevator, DoubleSupplier percentSupplier) {
    return new ManualElevatorControl(elevator, percentSupplier);
  }
}
