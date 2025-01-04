package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.GeometryUtils;

public class DriveSubsystem extends SubsystemBase {
    AHRS navx;

    private final Field2d _field = new Field2d();
    @SuppressWarnings({"FieldCanBeLocal", "FieldMayBeFinal"})
    private double throttleMultiplier = 1.0;
    private ChassisSpeeds lastSetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private double previousTargetHeading = 0;

    private final SwerveModule frontLeft =
      new SwerveModule(
          Constants.Swerve.FRONT_LEFT_DRIVING_CAN_ID,
          Constants.Swerve.FRONT_LEFT_TURNING_CAN_ID,
          Preferences.getDouble(
              Constants.Swerve.FRONT_LEFT_OFFSET_KEY,
              Constants.Swerve.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET_RAD),
          Constants.Swerve.FRONT_LEFT_IS_INVERTED);

    private final SwerveModule frontRight =
        new SwerveModule(
            Constants.Swerve.FRONT_RIGHT_DRIVING_CAN_ID,
            Constants.Swerve.FRONT_RIGHT_TURNING_CAN_ID,
            Preferences.getDouble(
                Constants.Swerve.FRONT_RIGHT_OFFSET_KEY,
                Constants.Swerve.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD),
            Constants.Swerve.FRONT_RIGHT_IS_INVERTED);

    private final SwerveModule rearLeft =
        new SwerveModule(
            Constants.Swerve.REAR_LEFT_DRIVING_CAN_ID,
            Constants.Swerve.REAR_LEFT_TURNING_CAN_ID,
            Preferences.getDouble(
                Constants.Swerve.REAR_LEFT_OFFSET_KEY,
                Constants.Swerve.REAR_LEFT_CHASSIS_ANGULAR_OFFSET_RAD),
            Constants.Swerve.REAR_LEFT_IS_INVERTED);

    private final SwerveModule rearRight =
        new SwerveModule(
            Constants.Swerve.REAR_RIGHT_DRIVING_CAN_ID,
            Constants.Swerve.REAR_RIGHT_TURNING_CAN_ID,
            Preferences.getDouble(
                Constants.Swerve.REAR_RIGHT_OFFSET_KEY,
                Constants.Swerve.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET_RAD),
            Constants.Swerve.REAR_RIGHT_IS_INVERTED);

    private PIDController rotController = new PIDController(
        Constants.Swerve.ROT_CONTROLLER_KP, 
        Constants.Swerve.ROT_CONTROLLER_KI,
        Constants.Swerve.ROT_CONTROLLER_KD
    );

    private SwerveDrivePoseEstimator odometry;

    private ChassisSpeeds actualChassisSpeed;

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return odometry;
    }

    public Field2d getField() {
        return _field;
    }

    public DriveSubsystem() {
        navx = new AHRS(NavXComType.kMXP_SPI);
        actualChassisSpeed = new ChassisSpeeds(0, 0, 0);
        navx.reset();

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        odometry = new SwerveDrivePoseEstimator(
                    Constants.Swerve.DRIVE_KINEMATICS,
                    Rotation2d.fromDegrees(this.getGyroYaw()),
                    this.getModulePositions(),
                    new Pose2d());

        // AutoBuilder.configureHolonomic(
        //     this::getPose,
        //     this::resetPose,
        //     this::getChassisSpeed,
        //     this::setPathFollowerSpeeds,
        //     Constants.PathFollowing.holonomicPathFollowerConfig,
        //     ()->false,
        //     this);
    }

    @Override
    public void periodic() {
      // Update the odometry in the periodic block
      frontLeft.periodic();
      frontRight.periodic();
      rearLeft.periodic();
      rearRight.periodic();
  
  
      odometry.update(Rotation2d.fromDegrees(getGyroYaw()), getModulePositions());
      actualChassisSpeed = Constants.Swerve.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  
      if (Constants.SHOW_2D_FIELD)
        _field.setRobotPose(getPose());
    }

    public double getGyroYaw() {
        return navx.getAngle() * (Constants.Swerve.GYRO_REVERSED ? -1 : 1);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
          frontLeft.getState(), frontRight.getState(), rearLeft.getState(), rearRight.getState()
        };
      }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        };
    }

      /**
   * Correction for swerve second order dynamics issue. Borrowed from 254:
   * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
   * Discussion:
   * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
   */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }


    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Desired speed of the robot in the x direction (forward), [-1,1].
   * @param ySpeed Desired speed of the robot in the y direction (sideways), [-1,1].
   * @param rot Desired angular rate of the robot, [-1,1].
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        // Adjust input based on max speed
        xSpeed *= Constants.Swerve.MAX_SPEED_METERS_PER_SECOND;
        ySpeed *= Constants.Swerve.MAX_SPEED_METERS_PER_SECOND;
        rot *= Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

        xSpeed *= throttleMultiplier;
        ySpeed *= throttleMultiplier;
        rot *= throttleMultiplier;

        ChassisSpeeds desiredChassisSpeeds =
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getGyroYaw()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
        //    This is the last commanded chassis speed not the last actual chassis speed
        this.lastSetChassisSpeeds = desiredChassisSpeeds;

        var swerveModuleStates =
            Constants.Swerve.DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECOND);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    private MedianFilter filter = new MedianFilter(5);

    public void driveWithHeading(double xSpeed, double ySpeed, Rotation2d targetHeadingRads, boolean fieldRelative){
        var delta = filter.calculate((targetHeadingRads.getRadians() - previousTargetHeading));

        rotController.setSetpoint(targetHeadingRads.getRadians() + delta*20);

        double rotControllerValue = rotController.calculate(
            this.getPose().getRotation().getRadians() % (Math.PI * 2)
            );

        drive(
            xSpeed,
            ySpeed,
            (rotControllerValue) / Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
            fieldRelative
        );

        previousTargetHeading = targetHeadingRads.getRadians();
    }

    public void turnToHeading(Rotation2d targeRotation2d){
        rotController.setSetpoint(targeRotation2d.getRadians());

        drive(
            0.0,
            0.0,
            rotController.calculate(
            this.getPose().getRotation().getRadians() % (Math.PI * 2)
            ) / Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS,
            false
        );
    }

    public void setPathFollowerSpeeds(ChassisSpeeds speeds) {
        setChassisSpeed(speeds);
    }

    public void setChassisSpeed(ChassisSpeeds speed) {
        double maxSpeed = Constants.Swerve.MAX_SPEED_METERS_PER_SECOND;
        double maxRotationalSpeed = Constants.Swerve.MAX_ANGULAR_SPEED_RAD_PER_SECONDS;

        drive(
            speed.vxMetersPerSecond / maxSpeed,
            speed.vyMetersPerSecond / maxSpeed,
            speed.omegaRadiansPerSecond / maxRotationalSpeed,
            false);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, Constants.Swerve.MAX_SPEED_METERS_PER_SECOND);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    public boolean atHeading(){
        return this.rotController.atSetpoint();
    }

    private boolean mirrorForRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
}
