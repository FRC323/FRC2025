package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class AlignToReefBranch extends Command {
  private final Drive drive;
  private final Vision vision;

  private final String logPrefix = Constants.ReefAlignmentConstants.smartDashboardLogPrefix;

  private int targetTagId = 0;
  private BranchSide branchSide;
  private Pose2d targetTagPose;
  private Pose2d desiredPose;
  private double tagScanStartTime = 0.0;

  private PIDController depthController;
  private PIDController lateralController;
  private PIDController rotationController;

  public enum BranchSide {
    LEFT,
    RIGHT
  }

  public AlignToReefBranch(Drive drive, Vision vision, int targetTagId, BranchSide branchSide) {
    this.drive = drive;
    this.vision = vision;

    this.targetTagId = targetTagId;
    this.branchSide = branchSide;

    depthController =
        new PIDController(
            Constants.ReefAlignmentConstants.depthP,
            Constants.ReefAlignmentConstants.depthI,
            Constants.ReefAlignmentConstants.depthD);
    depthController.setTolerance(Constants.ReefAlignmentConstants.depthThreshold);

    lateralController =
        new PIDController(
            Constants.ReefAlignmentConstants.lateralP,
            Constants.ReefAlignmentConstants.lateralI,
            Constants.ReefAlignmentConstants.lateralD);
    lateralController.setTolerance(Constants.ReefAlignmentConstants.lateralThreshold);

    rotationController =
        new PIDController(
            Constants.ReefAlignmentConstants.rotationP,
            Constants.ReefAlignmentConstants.rotationI,
            Constants.ReefAlignmentConstants.rotationD);
    rotationController.setTolerance(Constants.ReefAlignmentConstants.rotationThreshold);
  }

  @Override
  public void initialize() {
    this.tagScanStartTime = Timer.getFPGATimestamp();
    Pose3d targetTagPose3d = vision.getAprilTagPose(targetTagId, 2);

    if (targetTagPose3d == null) {
      writeMsgToSmartDashboard("Target tag pose not found for tag ID: " + targetTagId);
      return;
    }

    this.desiredPose = getDesiredPose(targetTagPose3d);
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    Pose3d targetTagPose3d = vision.getAprilTagPose(targetTagId, 2);
    if (targetTagPose3d != null) {
      this.desiredPose = getDesiredPose(targetTagPose3d);
    } else if (desiredPose == null) {
      if (currentTime - tagScanStartTime
          > Constants.ReefAlignmentConstants.tagScanTimeoutInSeconds) {
        writeMsgToSmartDashboard("Failed to get desired pose within 2 seconds. Ending command.");
        drive.runVelocity(new ChassisSpeeds(0, 0, 0));
        return;
      }
      writeMsgToSmartDashboard("Target tag pose not found for tag ID: " + targetTagId);
    }

    if (desiredPose != null) {
      writeMsgToSmartDashboard(
          "Desired pose calculated: x: "
              + desiredPose.getX()
              + " y: "
              + desiredPose.getY()
              + " rotation: "
              + desiredPose.getRotation().getDegrees());
      Pose2d drivePose = drive.getPose();

      Translation2d error = calculateError(desiredPose, drivePose);
      double xError = error.getX();
      double yError = error.getY();

      double xOutput = depthController.calculate(0, xError);
      double yOutput = lateralController.calculate(0, yError);
      double rotationOutput =
          rotationController.calculate(
              drivePose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

      normalizeVelocity(Constants.ReefAlignmentConstants.maxVelocity, xOutput, yOutput);

      ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(xOutput, yOutput, rotationOutput);
      ChassisSpeeds robotRelativeSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, drivePose.getRotation());

      if (Constants.ReefAlignmentConstants.moveRobot) drive.runVelocity(robotRelativeSpeeds);

      // Logging ...
      SmartDashboard.putNumber(logPrefix + "DesiredPoseX", desiredPose.getX());
      SmartDashboard.putNumber(logPrefix + "DesiredPoseY", desiredPose.getY());
      SmartDashboard.putNumber(logPrefix + "DrivePoseX", drivePose.getX());
      SmartDashboard.putNumber(logPrefix + "DrivePoseY", drivePose.getY());
      SmartDashboard.putNumber(logPrefix + "TargetTagPoseX", targetTagPose.getX());
      SmartDashboard.putNumber(logPrefix + "TargetTagPoseY", targetTagPose.getY());
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

    if (this.desiredPose == null || this.targetTagPose == null) {
      writeMsgToSmartDashboard("Desired pose or target tag pose is null. Cannot check alignment.");
      return false;
    }

    Pose2d drivePose = drive.getPose();
    boolean depthInPosition = depthInPosition(drivePose, desiredPose);
    boolean lateralInPosition = lateralInPosition(drivePose, desiredPose);
    boolean rotationInPosition = rotationInPosition(drivePose, desiredPose);

    if (depthInPosition && lateralInPosition && rotationInPosition) {
      writeMsgToSmartDashboard("Alignment complete. Ending command.");
      return true;
    } else {

      writeMsgToSmartDashboard(
          "Alignment not complete. Continuing execution. "
              + "Depth: "
              + depthInPosition
              + ", Lateral: "
              + lateralInPosition
              + ", Rotation: "
              + rotationInPosition);
      return false;
    }
  }

  private Pose2d getDesiredPose(Pose3d targetTagPose3d) {
    this.targetTagPose = targetTagPose3d.toPose2d();

    // robot offset from tag robot width / 2 basically
    double outwardOffset = Constants.ReefAlignmentConstants.outwardOffsetFromTag;

    // rotation
    double rotationTargetDegrees =
        Constants.ReefAlignmentConstants.tagToRotationTarget.get(targetTagId);
    double targetRotation = Units.degreesToRadians(rotationTargetDegrees);

    Alliance tagAlliance =
        Constants.ReefAlignmentConstants.tagToAlliance.getOrDefault(targetTagId, Alliance.Blue);

    // center of the reef based on alliance
    Translation2d reefCenter = Constants.ReefAlignmentConstants.blueReefCenter;
    if (tagAlliance == Alliance.Red) {
      reefCenter = Constants.ReefAlignmentConstants.redReefCenter;
    }

    // translation based on outward direction from reef center
    Translation2d tagTranslation = targetTagPose.getTranslation();
    Translation2d outwardDirection = tagTranslation.minus(reefCenter);
    outwardDirection = outwardDirection.div(outwardDirection.getNorm());

    // rotate outwardDirection by pi/2 radians to get the perpendicular direction
    Translation2d perpendicularDirection =
        outwardDirection.rotateBy(Rotation2d.fromRadians(Math.PI / 2));

    // Adjust offsets based on branch side
    double poleOffset = getPoleOffset(targetTagId, branchSide);
    Translation2d baseOffset = outwardDirection.times(outwardOffset);
    Translation2d poleSpecificOffset =
        perpendicularDirection.times(poleOffset * (branchSide == BranchSide.LEFT ? -1 : 1));

    Translation2d totalOffset = baseOffset.plus(poleSpecificOffset);
    Translation2d desiredTranslation = tagTranslation.plus(totalOffset);
    return new Pose2d(desiredTranslation, new Rotation2d(targetRotation));
  }

  private void normalizeVelocity(double maxVelocity, double xOutput, double yOutput) {
    double magnitude = Math.sqrt(xOutput * xOutput + yOutput * yOutput);

    if (magnitude > maxVelocity) {
      xOutput = (xOutput / magnitude) * maxVelocity;
      yOutput = (yOutput / magnitude) * maxVelocity;
    }
  }

  private Translation2d calculateError(Pose2d desiredPose, Pose2d drivePose) {
    return desiredPose.getTranslation().minus(drivePose.getTranslation());
  }

  private double getPoleOffset(int tagId, BranchSide branchSide) {
    String key = tagId + "_" + branchSide.name().toLowerCase();
    return Constants.ReefAlignmentConstants.tagToPoleOffset.getOrDefault(key, 0.33);
  }

  private boolean depthInPosition(Pose2d drivePose, Pose2d desiredPose) {
    return drivePose.getTranslation().getX()
            >= desiredPose.getTranslation().getX() - Constants.ReefAlignmentConstants.depthThreshold
        && drivePose.getTranslation().getX()
            <= desiredPose.getTranslation().getX()
                + Constants.ReefAlignmentConstants.depthThreshold;
  }

  private boolean lateralInPosition(Pose2d drivePose, Pose2d desiredPose) {
    return drivePose.getTranslation().getY()
            >= desiredPose.getTranslation().getY()
                - Constants.ReefAlignmentConstants.lateralThreshold
        && drivePose.getTranslation().getY()
            <= desiredPose.getTranslation().getY()
                + Constants.ReefAlignmentConstants.lateralThreshold;
  }

  private boolean rotationInPosition(Pose2d drivePose, Pose2d desiredPose) {
    return drivePose.getRotation().getRadians()
            >= desiredPose.getRotation().getRadians()
                - Constants.ReefAlignmentConstants.rotationThreshold
        && drivePose.getRotation().getRadians()
            <= desiredPose.getRotation().getRadians()
                + Constants.ReefAlignmentConstants.rotationThreshold;
  }

  private void writeMsgToSmartDashboard(String message) {
    SmartDashboard.putString(logPrefix + "Message", message);
    System.out.println(message);
  }
}

// pathplanner find path code
// private Command pathCommand;
// executre
// drive.findPathToPose(this.desiredPose).execute();
// if (pathCommand != null) {
// pathCommand.execute();
// } else {
// System.out.println("Path command is null. Cannot execute alignment.");
// return;
// }

// isfinished
// if (pathCommand != null && pathCommand.isFinished()) {
// pathCommand.end(false);
// return true;
// }
// return false;
// }

// end
// if (pathCommand != null) {
// pathCommand.end(interrupted);
// }

// init
// drive.setPose(desiredPose);

// pathCommand = drive.findPathToPose(desiredPose);
// pathCommand.initialize();
