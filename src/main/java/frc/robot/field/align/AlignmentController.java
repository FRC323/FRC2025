package frc.robot.field.align;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class AlignmentController {
  public static class Constants {
    public static final double depthP = 2;
    public static final double depthI = 0.0;
    public static final double depthD = 0.0;
    public static final double depthFF = 0.0;

    public static final double lateralP = 2;
    public static final double lateralI = 0.0;
    public static final double lateralD = 0.0;

    public static final double rotationP = 8.0;
    public static final double rotationI = 0.0;
    public static final double rotationD = 0.0;
    public static final double rotationFF = 0.0;

    public static final double rotationMaxVelocity = 3.0;
    public static final double rotationMaxAcceleration = 3.0;

    public static final double xTolerance = 0.05;
    public static final double yTolerance = 0.05;
    public static final double rotationTolerance = 0.05;
  }

  private final HolonomicDriveController hdcontroller;

  public AlignmentController() {
    PIDController depthController =
        new PIDController(Constants.depthP, Constants.depthI, Constants.depthD);
    PIDController lateralController =
        new PIDController(Constants.lateralP, Constants.lateralI, Constants.lateralD);
    ProfiledPIDController rotationController =
        new ProfiledPIDController(
            Constants.rotationP,
            Constants.rotationI,
            Constants.rotationD,
            new Constraints(Constants.rotationMaxVelocity, Constants.rotationMaxAcceleration));

    hdcontroller =
        new HolonomicDriveController(depthController, lateralController, rotationController);
    hdcontroller.setTolerance(
        new Pose2d(
            Constants.xTolerance,
            Constants.yTolerance,
            Rotation2d.fromDegrees(Constants.rotationTolerance)));
  }

  public HolonomicDriveController get() {
    return hdcontroller;
  }
}
