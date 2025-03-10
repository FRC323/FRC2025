package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import java.util.function.DoubleSupplier;

public class RunCoralIntakeWithScaling extends Command {
  private CoralIntake intake;
  private Arm arm;
  private DoubleSupplier normalSpeed;
  private DoubleSupplier armMovingSpeed;

  public RunCoralIntakeWithScaling(
      CoralIntake intake, Arm arm, DoubleSupplier normal, DoubleSupplier armMoving) {
    addRequirements(intake);
    this.intake = intake;
    this.arm = arm;
    this.normalSpeed = normal;
    this.armMovingSpeed = armMoving;
  }

  @Override
  public void execute() {
    double percent = normalSpeed.getAsDouble();
    intake.runPercentOutput(percent);
  }
}
