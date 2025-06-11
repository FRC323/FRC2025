package frc.robot.commands.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.field.TagReefPole;
import frc.robot.field.align.Reef;
import frc.robot.field.align.ReefAlignmentConstants;
import frc.robot.field.align.ReefAlignmentConstants.PoleSide;
import frc.robot.field.align.ReefAlignmentConstants.ReefPoleLabel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignToReefBranch extends Command {
  private final Drive drive;
  private final Vision vision;
  private final ReefPoleLabel pole;

  private final String logPrefix = ReefAlignmentConstants.smartDashboardLogPrefix;

  private int targetTagId = 0;
  private PoleSide poleSide;
  private Pose2d targetTagPose;
  private Pose2d desiredRobotPose;
  private double tagScanStartTime = 0.0;

  private HolonomicDriveController hdcontroller;
  private PIDController depthController;
  private PIDController lateralController;
  private ProfiledPIDController rotationController;

  public AlignToReefBranch(Drive drive, Vision vision, ReefPoleLabel pole) {
    this.drive = drive;
    this.vision = vision;
    this.pole = pole;

    depthController =
        new PIDController(
            ReefAlignmentConstants.depthP,
            ReefAlignmentConstants.depthI,
            ReefAlignmentConstants.depthD);

    lateralController =
        new PIDController(
            ReefAlignmentConstants.lateralP,
            ReefAlignmentConstants.lateralI,
            ReefAlignmentConstants.lateralD);

    rotationController =
        new ProfiledPIDController(
            ReefAlignmentConstants.rotationP,
            ReefAlignmentConstants.rotationI,
            ReefAlignmentConstants.rotationD,
            new Constraints(ReefAlignmentConstants.rotationMaxVelocity, ReefAlignmentConstants.rotationMaxAcceleration));

    hdcontroller =
        new HolonomicDriveController(depthController, lateralController, rotationController);
    hdcontroller.setTolerance(
        new Pose2d(
            ReefAlignmentConstants.xTolerance,
            ReefAlignmentConstants.yTolerance,
            Rotation2d.fromDegrees(ReefAlignmentConstants.rotationTolerance)));
  }

  @Override
  public void initialize() {
    this.tagScanStartTime = Timer.getFPGATimestamp();

    TagReefPole reefPole = Reef.getPoleFromLabel(pole, DriverStation.getAlliance());

    this.targetTagId = reefPole.tagId;
    this.poleSide = reefPole.poleSide;

    System.out.println(
        "AlignToReefBranch initialized with pole: "
            + pole
            + ", tag ID: "
            + reefPole.tagId
            + ", side: "
            + reefPole.poleSide);
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    Pose3d targetTagPose3d = null;
    targetTagPose3d = vision.getAprilTagPose(targetTagId, 2);
    if (targetTagPose3d == null) {
      targetTagPose3d = vision.getAprilTagPose(targetTagId, 1);
    }
    if (targetTagPose3d == null) {
      targetTagPose3d = vision.getAprilTagPose(targetTagId, 0);
    }
    if (targetTagPose3d != null) {
      this.targetTagPose = targetTagPose3d.toPose2d();
      this.desiredRobotPose =
          Reef.getReefPolePose(targetTagId, targetTagPose3d.toPose2d(), drive.getPose(), poleSide);
    } else if (desiredRobotPose == null) {
      if (currentTime - tagScanStartTime > ReefAlignmentConstants.tagScanTimeoutInSeconds) {
        writeMsgToSmartDashboard("Failed to get desired pose within 2 seconds. Ending command.");
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
        return;
      }
      writeMsgToSmartDashboard("Target tag pose not found for tag ID: " + targetTagId);
    }

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
          hdcontroller.calculate(drivePose, desiredRobotPose, 0.0, desiredRobotPose.getRotation());

      if (ReefAlignmentConstants.moveRobot) drive.runVelocity(speeds);

      // Logging ...
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

    boolean inPosition = hdcontroller.atReference();

    if (inPosition) {
      writeMsgToSmartDashboard("Alignment complete. Ending command.");
      return true;
    } else {

      writeMsgToSmartDashboard(
          "Alignment not complete. Continuing execution. "
              + "Depth: "
              + hdcontroller.getXController().atSetpoint()
              + ", Lateral: "
              + hdcontroller.getYController().atSetpoint()
              + ", Rotation: "
              + hdcontroller.getThetaController().atSetpoint());
      return false;
    }
  }

  private void writeMsgToSmartDashboard(String message) {
    SmartDashboard.putString(logPrefix + "Message", message);
    System.out.println(message);
  }
}
