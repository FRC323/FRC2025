package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import java.util.function.DoubleSupplier;

public class ManualCoralIntakeControl extends Command {
  private CoralIntake intake;
  private DoubleSupplier percentSupplier;

  public ManualCoralIntakeControl(CoralIntake intake, DoubleSupplier percentSupplier) {
    addRequirements(intake);
    this.intake = intake;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();
    intake.runPercentOutput(percent);
  }
}
