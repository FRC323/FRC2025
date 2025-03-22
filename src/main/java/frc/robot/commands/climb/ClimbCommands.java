package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.ground.GroundIntake;

public class ClimbCommands {
  public static Command ClimberZero(Climber climber) {
    return new ClimberZero(climber);
  }

  public static Command ClimberStop(Climber climber) {
    return new ClimberStop(climber);
  }

  public static Command MoveClimberDown(Climber climber) {
    return new RunClimbDown(climber);
  }

  public static Command MoveClimberUp(Climber climber) {
    return new RunClimbUp(climber);
  }

  public static Command MoveClimberToDeploy(
      Climber climber, Elevator elevator, Arm arm, GroundIntake groundIntake) {
    return new MoveToDeployClimber(climber, elevator, arm, groundIntake);
  }

  public static Command MoveClimberToDonkeyKong(Climber climber) {
    return new MoveToDonkeyKong(climber);
  }

  public static Command MoveClimberToStow(Climber climber) {
    return new MoveToStowClimber(climber);
  }
}
