package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import java.util.function.DoubleSupplier;

public class ArmCommands {
  public static Command moveArmToHome(Arm arm) {
    return new MoveArmToHome(arm);
  }

  public static Command moveArmToPosition(Arm arm, ArmPosition position) {
    return new MoveArmToPosition(arm, position);
  }

  public static Command manualArmControl(Arm arm, DoubleSupplier percentSupplier) {
    return new ManualArmControl(arm, percentSupplier);
  }

  public static Command moveArm45F(Arm arm) {
    return new MoveArm45F(arm);
  }

  public static Command moveArm45R(Arm arm) {
    return new MoveArm45R(arm);
  }
}
