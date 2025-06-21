package frc.robot.field.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.ReefPoleOffset;
import frc.robot.field.TagReefPole;
import frc.robot.field.align.ReefAlignmentConstants.PoleSide;
import frc.robot.field.align.ReefAlignmentConstants.ReefPoleLabel;
import java.util.Optional;

public class Reef {

  private static Pose2d getReefTagPoseWithOffset(
      int targetTagId, Pose2d targetTagPose, Pose2d robotPose, double offset) {

    // determine robot location from tag
    Pose2d tagRelativeToRobot = targetTagPose.relativeTo(robotPose);
    Transform2d offsetFromTag =
        new Transform2d(
            new Translation2d(ReefAlignmentConstants.outwardOffsetFromTag, offset),
            new Rotation2d());
    Pose2d targetRelativeToRobot = tagRelativeToRobot.plus(offsetFromTag);

    Pose2d targetInField = robotPose.plus(new Transform2d(new Pose2d(), targetRelativeToRobot));

    // predefined rotation
    Rotation2d targetRotation =
        Rotation2d.fromDegrees(ReefAlignmentConstants.tagToRotationTarget.get(targetTagId));

    return new Pose2d(targetInField.getTranslation(), targetRotation);
  }

  public static Pose2d getReefPolePose(
      int targetTagId, Pose2d tagetTagPose, Pose2d robotPose, PoleSide poleSide) {
    // pole offsets from tag
    double poleOffsetFromTag = getTagToPoleOffset(targetTagId, poleSide);
    return getReefTagPoseWithOffset(targetTagId, tagetTagPose, robotPose, poleOffsetFromTag);
  }

  public static double estimatedDistance(Pose2d drivePose, Pose2d targetTagPose) {
    return drivePose.getTranslation().getDistance(targetTagPose.getTranslation());
  }

  public static double getTagToPoleOffset(int tagId, PoleSide poleSide) {
    ReefPoleOffset offsets = ReefAlignmentConstants.tagToPoleOffset.get(tagId);
    if (offsets == null) return 0.0;
    return poleSide == PoleSide.LEFT ? -offsets.left : offsets.right;
  }

  public static ReefPoleLabel getPoleLabelFromTagId(int tagId, PoleSide poleSide) {
    for (ReefPoleLabel label : ReefPoleLabel.values()) {
      TagReefPole pole = getPoleFromLabel(label, Optional.empty());
      if (pole.tagId == tagId && pole.poleSide == poleSide) {
        return label;
      }
    }
    throw new IllegalArgumentException(
        "No ReefPoleLabel found for tagId: " + tagId + " and poleSide: " + poleSide);
  }

  public static TagReefPole getPoleFromLabel(ReefPoleLabel poleLabel, Optional<Alliance> alliance) {
    if (alliance.orElse(Alliance.Blue) == Alliance.Red) {
      if (poleLabel == ReefPoleLabel.A) return new TagReefPole(7, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.B) return new TagReefPole(7, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.C) return new TagReefPole(8, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.D) return new TagReefPole(8, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.E) return new TagReefPole(9, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.F) return new TagReefPole(9, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.G) return new TagReefPole(10, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.H) return new TagReefPole(10, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.I) return new TagReefPole(11, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.J) return new TagReefPole(11, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.K) return new TagReefPole(6, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.L) return new TagReefPole(6, PoleSide.RIGHT);
    } else {
      if (poleLabel == ReefPoleLabel.A) return new TagReefPole(18, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.B) return new TagReefPole(18, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.C) return new TagReefPole(17, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.D) return new TagReefPole(17, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.E) return new TagReefPole(22, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.F) return new TagReefPole(22, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.G) return new TagReefPole(21, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.H) return new TagReefPole(21, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.I) return new TagReefPole(20, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.J) return new TagReefPole(20, PoleSide.RIGHT);
      if (poleLabel == ReefPoleLabel.K) return new TagReefPole(19, PoleSide.LEFT);
      if (poleLabel == ReefPoleLabel.L) return new TagReefPole(19, PoleSide.RIGHT);
    }
    throw new IllegalArgumentException("Invalid ReefPoleLabel: " + poleLabel);
  }
}
