// package frc.robot.commands.auto;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;

// /**
//  * Command to drive the robot left at 25% speed for 2 seconds
//  */
// public class AutoD extends Command {
//   private final Drive drive;
//   private final double speed;
//   private final double duration;
//   private long startTime;
  
//   /**
//    * Creates a new DriveLeft command
//    * 
//    * @param drive The drive subsystem
//    */
//   public DriveLeft(Drive drive) {
//     this.drive = drive;
//     this.speed = 0.25; // 25% of max speed
//     this.duration = 2.0; // 2 seconds
//     addRequirements(drive);
//   }
  
//   @Override
//   public void initialize() {
//     startTime = System.currentTimeMillis();
//   }
  
//   @Override
//   public void execute() {
//     // Create robot-relative chassis speeds (positive y is left)
//     double yVelocity = speed * drive.getMaxLinearSpeedMetersPerSec();
//     ChassisSpeeds speeds = new ChassisSpeeds(0.0, yVelocity, 0.0);
    
//     // Apply speeds to drive subsystem
//     drive.runVelocity(speeds);
//   }
  
//   @Override
//   public boolean isFinished() {
//     // Check if we've exceeded the duration
//     return (System.currentTimeMillis() - startTime) >= (duration * 1000);
//   }
  
//   @Override
//   public void end(boolean interrupted) {
//     // Stop the drive when the command ends
//     drive.stop();
//   }
// }