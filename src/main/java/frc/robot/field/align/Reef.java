package frc.robot.field.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.field.ReefPoleOffset;
import frc.robot.field.TagReefPole;
import frc.robot.field.align.ReefAlignConstants.ReefAlignmentConstants.PoleSide;
import frc.robot.field.align.ReefAlignConstants.ReefAlignmentConstants.ReefPoleLabel;
import java.util.Optional;

public class Reef {
  public static Pose2d getReefPolePose(int targetTagId, Pose3d targetTagPose3d, PoleSide poleSide) {
    Pose2d targetTagPose = targetTagPose3d.toPose2d();

    // robot offset from tag robot width / 2 basically
    double outwardOffset = ReefAlignConstants.ReefAlignmentConstants.outwardOffsetFromTag;

    // rotation
    double rotationTargetDegrees =
        ReefAlignConstants.ReefAlignmentConstants.tagToRotationTarget.get(targetTagId);
    double targetRotation = Units.degreesToRadians(rotationTargetDegrees);

    Alliance tagAlliance =
        ReefAlignConstants.ReefAlignmentConstants.tagToAlliance.getOrDefault(
            targetTagId, Alliance.Blue);

    // center of the reef based on alliance
    Translation2d reefCenter = ReefAlignConstants.ReefAlignmentConstants.blueReefCenter;
    if (tagAlliance == Alliance.Red) {
      reefCenter = ReefAlignConstants.ReefAlignmentConstants.redReefCenter;
    }

    // translation based on outward direction from reef center
    Translation2d tagTranslation = targetTagPose.getTranslation();
    Translation2d outwardDirection = tagTranslation.minus(reefCenter);
    outwardDirection = outwardDirection.div(outwardDirection.getNorm());

    // rotate outwardDirection by pi/2 radians to get the perpendicular direction
    Translation2d perpendicularDirection =
        outwardDirection.rotateBy(Rotation2d.fromRadians(Math.PI / 2));

    // Adjust offsets based on branch side
    double poleOffset = getTagToPoleOffset(targetTagId, poleSide);
    Translation2d baseOffset = outwardDirection.times(outwardOffset);
    Translation2d poleSpecificOffset =
        perpendicularDirection.times(poleOffset * (poleSide == PoleSide.LEFT ? -1 : 1));

    Translation2d totalOffset = baseOffset.plus(poleSpecificOffset);
    Translation2d desiredTranslation = tagTranslation.plus(totalOffset);
    return new Pose2d(desiredTranslation, new Rotation2d(targetRotation));
  }

  public static double getTagToPoleOffset(int tagId, PoleSide poleSide) {
    ReefPoleOffset offsets = ReefAlignConstants.ReefAlignmentConstants.tagToPoleOffset.get(tagId);
    if (offsets == null) return 0.0;
    return poleSide == PoleSide.LEFT ? offsets.left : offsets.right;
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
