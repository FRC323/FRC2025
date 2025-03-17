package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;

public class CommonCommands {
  public static Command moveToTravelPosition(
      Elevator elevator, Arm arm, CoralIntake coralIntake, AlgaeIntake algaeIntake) {
    return new MoveToTravel(elevator, arm, coralIntake, algaeIntake);
  }
}
