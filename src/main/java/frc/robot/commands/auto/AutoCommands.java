package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.intake.RunCoralIntake;
import frc.robot.commands.intake.RunCoralOuttake;
import frc.robot.field.align.ReefAlignmentConstants.ReefPoleLabel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.vision.Vision;

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

  public static Command AlignToReef(Drive drive, Vision vision, ReefPoleLabel pole) {
    return new AlignToReefBranch(drive, vision, pole).withTimeout(5);
  }

  public static Command AlignToCoralStation12(Drive drive, Vision vision) {
    return new AlignToCoralStation(drive, vision, 12).withTimeout(5);
  }
}
