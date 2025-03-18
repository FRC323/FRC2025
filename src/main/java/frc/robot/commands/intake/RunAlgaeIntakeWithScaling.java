package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import java.util.function.DoubleSupplier;

public class RunAlgaeIntakeWithScaling extends Command {
  private AlgaeIntake intake;
  private Arm arm;
  private Elevator elevator;
  private DoubleSupplier normalSpeed;
  private DoubleSupplier armMovingSpeed;

  public RunAlgaeIntakeWithScaling(
      AlgaeIntake intake,
      Arm arm,
      Elevator elevator,
      DoubleSupplier normal,
      DoubleSupplier armMoving) {
    addRequirements(intake);
    this.intake = intake;
    this.arm = arm;
    this.elevator = elevator;
    this.normalSpeed = normal;
    this.armMovingSpeed = armMoving;
  }

  @Override
  public void execute() {
    double percent = normalSpeed.getAsDouble();

    var elevatorNearHome =
        MathUtil.isNear(Elevator.ElevatorPosition.Home.val, elevator.getPosition(), .1);
    var armNearHome = MathUtil.isNear(Arm.ArmPosition.HOME.val, arm.getAngleRadians(), .1);

    if (elevatorNearHome && armNearHome) {
      percent = 0;
    }
    intake.runPercentOutput(percent);
  }
}
