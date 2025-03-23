package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import java.util.function.DoubleSupplier;

public class HoldCoralAndDetectScore extends Command {
  private CoralIntake intake;
  private Elevator elevator;
  private Arm arm;
  private DoubleSupplier percentSupplier;

  public HoldCoralAndDetectScore(
      CoralIntake intake, Elevator elevator, Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(intake);
    this.intake = intake;
    this.elevator = elevator;
    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void execute() {
    if (elevator.isInReefScoringPosition()
        && arm.isInReefScoringPosition()
        && intake.isReefBranchDetected()) {
      intake.runPercentOutput(-CoralIntakeConstants.IntakeOutput);
    } else {
      double percent = percentSupplier.getAsDouble();
      intake.runPercentOutput(percent);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
