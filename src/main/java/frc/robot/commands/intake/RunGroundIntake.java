package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import java.util.function.DoubleSupplier;

public class RunGroundIntake extends Command {
  private GroundIntake intake;
  private DoubleSupplier percentSupplier;

  public RunGroundIntake(GroundIntake intake, DoubleSupplier percentSupplier) {
    addRequirements(intake);
    this.intake = intake;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();
    intake.runIntakePercentOutput(percent);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
