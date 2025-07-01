package frc.robot.commands.common;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.ground.GroundIntake;

public class CommonCommands {
  public static Command moveToTravelPosition(
      Elevator elevator,
      Arm arm,
      CoralIntake coralIntake,
      AlgaeIntake algaeIntake,
      GroundIntake groundIntake) {
    return new MoveToTravel(elevator, arm, coralIntake, algaeIntake, groundIntake);
  }

  public static Command RotateToAngle(Drive drive, double targetAngle) {
    return new RotateToAngle(drive, targetAngle);
  }
}
