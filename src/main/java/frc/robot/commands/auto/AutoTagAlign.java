// package frc.robot.commands.auto;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.vision.Vision;

// /** Aligns robot to an AprilTag by driving sideways and rotating to face tag with robot's side */
// public class AutoTagAlign extends Command {
//   private final Drive drive;
//   private final Vision vision;
//   private final int cameraIndex;
//   private final Timer timer = new Timer();

//   // PID Controllers
//   private final PIDController alignController = new PIDController(0.03, 0, 0);

//   /** Creates a command to align with an AprilTag */
//   public AutoTagAlign(Drive drive, Vision vision, int cameraIndex) {
//     this.drive = drive;
//     this.vision = vision;
//     this.cameraIndex = cameraIndex;
//     addRequirements(drive);
//   }

//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();

//     vision.setPipeline(0, 1);
//   }

//   @Override
//   public void execute() {
//     // Get angle to target from vision
//     double tagYaw = vision.getTargetX(cameraIndex).getDegrees();

//     // If no tag visible, stop
//     if (Double.isNaN(tagYaw)) {
//       drive.stop();
//       return;
//     }

//     // Calculate lateral speed (positive yaw = tag to right, drive left)
//     double lateralSpeed = -alignController.calculate(tagYaw, 0);

//     // Limit speed to 25% of max
//     lateralSpeed = Math.max(-0.25, Math.min(0.25, lateralSpeed));

//     // Set robot speeds (lateral movement only)
//     ChassisSpeeds speeds =
//         new ChassisSpeeds(
//             0.0, // No forward/back
//             lateralSpeed * drive.getMaxLinearSpeedMetersPerSec(),
//             0.0); // No rotation

//     drive.runVelocity(speeds);
//   }

//   @Override
//   public boolean isFinished() {
//     // End if aligned or timed out
//     return Math.abs(vision.getTargetX(cameraIndex).getDegrees()) < 2.0 || timer.hasElapsed(3.0);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drive.stop();
//     vision.setPipeline(0, 0);
//   }
// }
