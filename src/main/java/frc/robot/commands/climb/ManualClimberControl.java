// package frc.robot.commands.climb;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.climber.Climber;
// import java.util.function.DoubleSupplier;

// public class ManualClimberControl extends Command {
//   private Climber climber;
//   private DoubleSupplier percentSupplier;

//   public ManualClimberControl(Climber climber, DoubleSupplier percentSupplier) {
//     addRequirements(climber);
//     this.climber = climber;
//     this.percentSupplier = percentSupplier;
//   }

//   @Override
//   public void execute() {
//     double percent = 0;
//     if (percentSupplier.getAsDouble() > .1 && percentSupplier.getAsDouble() < .1) {
//       percent = percentSupplier.getAsDouble();
//     } else {
//       percent = 0;
//     }
//     climber.runPercentOutput(percent);
//   }
// }
