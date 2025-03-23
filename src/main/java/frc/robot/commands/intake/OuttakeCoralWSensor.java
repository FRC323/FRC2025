package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeConstants;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntakeConstants;

public class OuttakeCoralWSensor extends SequentialCommandGroup {
  public OuttakeCoralWSensor(
      CoralIntake coralIntake, GroundIntake groundIntake, Elevator elevator, Arm arm) {
    addCommands(
        new RepeatCommand(
            new ConditionalCommand(
                new ParallelCommandGroup(
                    new RunCoralIntake(coralIntake, () -> -CoralIntakeConstants.IntakeOutput),
                    new RunGroundIntake(groundIntake, () -> -GroundIntakeConstants.IntakeOutput)),
                new InstantCommand(),
                () ->
                    coralIntake.isReefBranchDetected()
                        || (elevator.elevatorInGroundIntakePosition()
                            && arm.armInGroundIntakePosition()))));
  }
}
