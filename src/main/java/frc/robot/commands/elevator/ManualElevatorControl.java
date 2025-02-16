package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ManualElevatorControl extends Command {
  private Elevator elevator;
  private DoubleSupplier percentSupplier;

  public ManualElevatorControl(Elevator elevator, DoubleSupplier percentSupplier) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();
    elevator.runPercentOutput(percent);
  }
}
