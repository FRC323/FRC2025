package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmCommands {
  public static Command moveArm45F(Arm arm) {
    return new MoveArm45F(arm);
  }

  public static Command moveArm45R(Arm arm) {
    return new MoveArm45R(arm);
  }
}
