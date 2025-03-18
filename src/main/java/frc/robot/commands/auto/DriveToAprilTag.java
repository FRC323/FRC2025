package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

public class DriveToAprilTag extends Command {
  private final Vision vision;
  private final Drive drive;
  private final int cameraIndex;
  private final int tagId;
  private final double stopDistance; // meters
  private final PathConstraints constraints;

  // Command state
  private boolean isPathfinding = false;
  private boolean isFinished = false;

  // TODO: make sure tag we're looking for is on the right
  // alliance. Also do we need to flip for red alliance?
  public DriveToAprilTag(
      Vision vision,
      Drive drive,
      int cameraIndex,
      int tagId,
      double stopDistance,
      double maxVelocity,
      double maxAcceleration) {

    this.vision = vision;
    this.drive = drive;
    this.cameraIndex = cameraIndex;
    this.tagId = tagId;
    this.stopDistance = stopDistance;
    this.constraints =
        new PathConstraints(
            maxVelocity, maxAcceleration, Units.degreesToRadians(540), Units.degreesToRadians(720));

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    isPathfinding = false;
    isFinished = false;
    SmartDashboard.putBoolean("DriveToAprilTag/Active", true);
  }

  @Override
  public void execute() {
    if (isPathfinding) {
      return;
    }

    // Get the tag pose
    Pose2d tagPose = getTagPose(tagId);
    if (tagPose == null) {
      drive.stop();
      return;
    }

    // Check if we've already reached the target distance
    Pose2d robotPose = drive.getPose();
    Translation2d toTag = tagPose.getTranslation().minus(robotPose.getTranslation());
    double distanceToTag = toTag.getNorm();

    if (distanceToTag <= stopDistance) {
      drive.stop();
      isFinished = true;
      return;
    }

    // Calculate the adjusted target pose with proper offset
    Translation2d direction = toTag.div(distanceToTag);
    Translation2d targetTranslation = tagPose.getTranslation().minus(direction.times(stopDistance));

    // Rotation2d leftSideAlignmentRotation = new Rotation2d(direction.getY(), -direction.getX());
    //Optional<Alliance> alliance = DriverStation.getAlliance();
    //boolean isRedAlliance = alliance.isPresent() && alliance.get() == Alliance.Red;

    // Use the tag's rotation for the target pose
    // add 90 degrees to the tag's rotation to align with the tag's left side
    Pose2d targetPose =
        new Pose2d(targetTranslation, tagPose.getRotation().plus(Rotation2d.fromDegrees(90)));

    // Output target pose to SmartDashboard
    SmartDashboard.putNumber("DriveToAprilTag/TargetX", targetPose.getX());
    SmartDashboard.putNumber("DriveToAprilTag/TargetY", targetPose.getY());
    SmartDashboard.putNumber("DriveToAprilTag/TargetRot", targetPose.getRotation().getDegrees());

    // Start pathfinding
    isPathfinding = true;

    // Use PathPlanner to drive to the target pose
    Command pathfindCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    pathfindCommand
        .andThen(
            () -> {
              isPathfinding = false;
              checkIfFinished(tagPose);
            })
        .schedule();
  }

  private void checkIfFinished(Pose2d tagPose) {
    Pose2d currentPose = drive.getPose();
    Translation2d currentToTag = tagPose.getTranslation().minus(currentPose.getTranslation());
    double currentDistanceToTag = currentToTag.getNorm();

    if (currentDistanceToTag <= stopDistance) {
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    SmartDashboard.putBoolean("DriveToAprilTag/Active", false);
  }

  private Pose2d getTagPose(int id) {
    var tagPose = vision.getAprilTagPose(id, cameraIndex);
    if (tagPose != null) {
      return tagPose.toPose2d();
    }
    return null;
  }
}
