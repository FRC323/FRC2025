package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import java.util.function.DoubleSupplier;

public class ManualArmControl extends Command {
  private Arm arm;
  private DoubleSupplier percentSupplier;

  public ManualArmControl(Arm arm, DoubleSupplier percentSupplier) {
    addRequirements(arm);
    this.arm = arm;
    this.percentSupplier = percentSupplier;
  }

  @Override
  public void execute() {
    double percent = percentSupplier.getAsDouble();
    arm.runPercentOutput(percent);
  }
}
