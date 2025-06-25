package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.align.AlignmentController;
import frc.robot.field.align.CoralStation;
import frc.robot.field.align.CoralStationAlignmentConstants;
import frc.robot.field.align.Reef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignToCoralStation extends Command {
  private final Drive drive;
  private final Vision vision;
  private final AlignmentController controller = new AlignmentController();
  private final String logPrefix = CoralStationAlignmentConstants.smartDashboardLogPrefix;

  private int targetTagId;
  private Pose2d targetTagPose;
  private Pose2d desiredRobotPose;
  private Pose2d initialRobotPose;
  private double calculatedDistance;

  public AlignToCoralStation(Drive drive, Vision vision, int targetTagId) {
    this.drive = drive;
    this.vision = vision;
    this.targetTagId = targetTagId;

    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    this.desiredRobotPose = null;
    this.initialRobotPose = drive.getPose();
    System.out.println("AlignToCoralStation initialized with tagId: " + this.targetTagId);

    Pose3d targetTagPose3d = null;
    targetTagPose3d = vision.getAprilTagPose(targetTagId, 1);

    if (targetTagPose3d != null) {
      this.targetTagPose = targetTagPose3d.toPose2d();
      this.desiredRobotPose =
          CoralStation.getStationPose(targetTagId, targetTagPose, drive.getPose());

      this.calculatedDistance = Reef.estimatedDistance(drive.getPose(), targetTagPose);

      writeMsgToSmartDashboard(
          "Found tag "
              + targetTagId
              + " at pose x: "
              + targetTagPose.getX()
              + ", y: "
              + targetTagPose.getY()
              + ", rotation: "
              + targetTagPose.getRotation().getDegrees()
              + ". Estimated distance: "
              + this.calculatedDistance);
    }
  }

  @Override
  public void execute() {
    if (desiredRobotPose != null) {
      writeMsgToSmartDashboard(
          "Desired pose calculated: x: "
              + desiredRobotPose.getX()
              + " y: "
              + desiredRobotPose.getY()
              + " rotation: "
              + desiredRobotPose.getRotation().getDegrees());
      Pose2d drivePose = drive.getPose();

      ChassisSpeeds speeds =
          controller
              .get()
              .calculate(drivePose, desiredRobotPose, 0.0, desiredRobotPose.getRotation());

      ChassisSpeeds scaledSpeeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * CoralStationAlignmentConstants.xyScalingFactor,
              speeds.vyMetersPerSecond * CoralStationAlignmentConstants.xyScalingFactor,
              speeds.omegaRadiansPerSecond * CoralStationAlignmentConstants.rotationScalingFactor);

      if (CoralStationAlignmentConstants.moveRobot) drive.runVelocity(scaledSpeeds);

      // Logging ...
      if (CoralStationAlignmentConstants.smartDashboardLogging) {
        SmartDashboard.putNumber(logPrefix + "DesiredPoseX", desiredRobotPose.getX());
        SmartDashboard.putNumber(logPrefix + "DesiredPoseY", desiredRobotPose.getY());
        SmartDashboard.putNumber(
            logPrefix + "DesiredPoseRotation", desiredRobotPose.getRotation().getDegrees());
        SmartDashboard.putNumber(logPrefix + "DrivePoseX", drivePose.getX());
        SmartDashboard.putNumber(logPrefix + "DrivePoseY", drivePose.getY());
        SmartDashboard.putNumber(
            logPrefix + "DrivePoseRotation", drivePose.getRotation().getDegrees());
        SmartDashboard.putNumber(logPrefix + "TargetTagPoseX", targetTagPose.getX());
        SmartDashboard.putNumber(logPrefix + "TargetTagPoseY", targetTagPose.getY());
        SmartDashboard.putNumber(
            logPrefix + "TargetTagPoseRotation", targetTagPose.getRotation().getDegrees());
        SmartDashboard.putNumber(logPrefix + "TargetTagId", targetTagId);
      }
    } else {
      writeMsgToSmartDashboard("Desired pose is null. Cannot execute alignment.");
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {

    if (this.desiredRobotPose == null || this.targetTagPose == null) {
      writeMsgToSmartDashboard("Desired pose or target tag pose is null. Cannot check alignment.");
      return false;
    }

    if (controller.isAligned()) {
      double traveledDistance =
          initialRobotPose.getTranslation().getDistance(this.drive.getPose().getTranslation());
      writeMsgToSmartDashboard(
          "Alignment complete. Ending command. Traveled distance: " + traveledDistance);
      return true;
    } else {

      writeMsgToSmartDashboard(
          "Alignment not complete. Continuing execution. "
              + "Depth: "
              + controller.get().getXController().atSetpoint()
              + ", Lateral: "
              + controller.get().getYController().atSetpoint()
              + ", Rotation: "
              + controller.get().getThetaController().atSetpoint());
      return false;
    }
  }

  private void writeMsgToSmartDashboard(String message) {
    if (!CoralStationAlignmentConstants.smartDashboardLogging) {
      return;
    }
    SmartDashboard.putString(logPrefix + "Message", message);
    System.out.println(message);
  }
}
