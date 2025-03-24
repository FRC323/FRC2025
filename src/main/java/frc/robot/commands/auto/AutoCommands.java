package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.intake.RunCoralIntake;
import frc.robot.commands.intake.RunCoralOuttake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;

public class AutoCommands {

  public static Command MoveToCoralIntakeAuto(
      Elevator elevator, Arm arm, CoralIntake coralIntake, GroundIntake groundIntake) {
    return new IntakeCoralAuto(elevator, arm, coralIntake, groundIntake);
  }

  public static Command CoralIntakeAuto(CoralIntake coralIntake) {
    return new RunCoralIntake(coralIntake, () -> CoralIntakeConstants.IntakeOutput).withTimeout(2);
  }

  public static Command CoralOuttakeAuto(CoralIntake coralIntake) {
    return new RunCoralOuttake(coralIntake, () -> CoralIntakeConstants.IntakeOutput).withTimeout(2);
  }
}
