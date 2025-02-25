package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

public class ScoreCommands {
  public static Command ScoreCoralL4(Elevator elevator, Arm arm) {
    return new ScoreCoralL4(elevator, arm);
  }

  public static Command ScoreCoralL3(Elevator elevator, Arm arm) {
    return new ScoreCoralL3(elevator, arm);
  }

  public static Command ScoreCoralL2(Elevator elevator, Arm arm) {
    return new ScoreCoralL2(elevator, arm);
  }
}
