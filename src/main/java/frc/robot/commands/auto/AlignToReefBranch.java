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

  private HolonomicDriveController hdcontroller;
  private PIDController depthController;
  private PIDController lateralController;
  private ProfiledPIDController rotationController;

  public AlignToReefBranch(Drive drive, Vision vision, ReefPoleLabel pole) {
    this.drive = drive;
    this.vision = vision;
    this.pole = pole;

    addRequirements(drive, vision);

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
            new Constraints(
                ReefAlignmentConstants.rotationMaxVelocity,
                ReefAlignmentConstants.rotationMaxAcceleration));

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
    TagReefPole reefPole = Reef.getPoleFromLabel(pole, DriverStation.getAlliance());

    this.targetTagId = reefPole.tagId;
    this.poleSide = reefPole.poleSide;

    Pose3d elevator_targetTagPose3d = null;
    elevator_targetTagPose3d = vision.getAprilTagPose(targetTagId, 2); // elevator camera
    Pose3d front_targetTagPose3d = null;
    front_targetTagPose3d = vision.getAprilTagPose(targetTagId, 0); // front
    Pose2d tagPose = null;

    if (elevator_targetTagPose3d != null && front_targetTagPose3d != null) {
      tagPose = averagePoses(elevator_targetTagPose3d, front_targetTagPose3d);
    } else if (elevator_targetTagPose3d != null) {
      tagPose = elevator_targetTagPose3d.toPose2d();
    } else if (front_targetTagPose3d != null) {
      tagPose = front_targetTagPose3d.toPose2d();
    }

    if (tagPose != null) {
      this.targetTagPose = tagPose;
      this.desiredRobotPose = Reef.getReefPolePose(targetTagId, tagPose, drive.getPose(), poleSide);
    }

    System.out.println(
        "AlignToReefBranch initialized with pole: "
            + pole
            + ", tag ID: "
            + reefPole.tagId
            + ", side: "
            + reefPole.poleSide);
  }

  private Pose2d averagePoses(Pose3d pose1, Pose3d pose2) {
    double avgX = (pose1.getX() + pose2.getX()) / 2.0;
    double avgY = (pose1.getY() + pose2.getY()) / 2.0;
    Rotation2d avgRotation =
        new Rotation2d(
            (pose1.getRotation().toRotation2d().getRadians()
                    + pose2.getRotation().toRotation2d().getRadians())
                / 2.0);
    return new Pose2d(avgX, avgY, avgRotation);
  }

  @Override
  public void execute() {
    if (this.desiredRobotPose != null) {
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

      ChassisSpeeds scaledSpeeds =
          new ChassisSpeeds(
              speeds.vxMetersPerSecond * ReefAlignmentConstants.xyScalingFactor,
              speeds.vyMetersPerSecond * ReefAlignmentConstants.xyScalingFactor,
              speeds.omegaRadiansPerSecond * ReefAlignmentConstants.rotationScalingFactor);

      if (ReefAlignmentConstants.moveRobot) drive.runVelocity(scaledSpeeds);

      // Logging ...
      if (ReefAlignmentConstants.smartDashboardLogging) {
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
    if (!ReefAlignmentConstants.smartDashboardLogging) {
      return;
    }
    SmartDashboard.putString(logPrefix + "Message", message);
    System.out.println(message);
  }
}
