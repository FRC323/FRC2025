package frc.robot.field;

import frc.robot.field.align.ReefAlignConstants.ReefAlignmentConstants.PoleSide;

public class TagReefPole {
  public final int tagId;
  public final PoleSide poleSide;

  public TagReefPole(int tagId, PoleSide poleSide) {
    this.tagId = tagId;
    this.poleSide = poleSide;
  }
}
