package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AlignToReefBranch.BranchSide;
import frc.robot.commands.intake.RunCoralIntake;
import frc.robot.commands.intake.RunCoralOuttake;
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

  public static Command AlignToReefBranch11(Drive drive, Vision vision) {
    return new AlignToReefBranch(drive, vision, 11, BranchSide.LEFT).withTimeout(5);
  }

  public static Command AlignToReefBranch8(Drive drive, Vision vision) {
    return new AlignToReefBranch(drive, vision, 8, BranchSide.LEFT).withTimeout(5);
  }

  public static Command AlignToReefBranch22(Drive drive, Vision vision) {
    return new AlignToReefBranch(drive, vision, 22, BranchSide.RIGHT).withTimeout(5);
  }

  public static Command AlignToReefBranch17(Drive drive, Vision vision) {
    return new AlignToReefBranch(drive, vision, 17, BranchSide.RIGHT).withTimeout(50);
  }

  public static Command AlignToReefBranch18(Drive drive, Vision vision) {
    return new AlignToReefBranch(drive, vision, 18, BranchSide.LEFT).withTimeout(5);
  }

  public static Command AlignToReefBranch20(Drive drive, Vision vision) {
    return new AlignToReefBranch(drive, vision, 20, BranchSide.LEFT).withTimeout(5);
  }
}
