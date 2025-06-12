package frc.robot.field.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CoralStation {
  public static Pose2d getStationPose(int targetTagId, Pose2d targetTagPose, Pose2d robotPose) {

    // determine robot location from tag
    Pose2d tagRelativeToRobot = targetTagPose.relativeTo(robotPose);

    Transform2d offsetFromTag =
        new Transform2d(
            new Translation2d(
                CoralStationAlignmentConstants.outwardOffsetFromTag,
                CoralStationAlignmentConstants.lateralOffsetFromTag),
            new Rotation2d());
    Pose2d targetRelativeToRobot = tagRelativeToRobot.plus(offsetFromTag);
    Pose2d targetInField = robotPose.plus(new Transform2d(new Pose2d(), targetRelativeToRobot));

    // predefined rotation
    Rotation2d targetRotation =
        Rotation2d.fromDegrees(CoralStationAlignmentConstants.tagToRotationTarget.get(targetTagId));

    return new Pose2d(targetInField.getTranslation(), targetRotation);
  }
}
