package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import java.util.function.DoubleSupplier;

public class RunCoralIntakeWithScaling extends Command {
  private CoralIntake intake;
  private Arm arm;
  private DoubleSupplier percentSupplier;

  public RunCoralIntakeWithScaling(CoralIntake intake, Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(intake);
    this.intake = intake;
    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();
    if (arm.isInInvertedRange() && percent == CoralIntakeConstants.normalOutput) {
      percent = CoralIntakeConstants.coralArmInvertedOutput;
    }
    intake.runPercentOutput(percent);
  }
}
