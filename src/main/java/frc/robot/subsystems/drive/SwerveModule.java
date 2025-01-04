package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.utils.AbsoluteEncoderChecker;
import frc.robot.utils.SparkMaxUtils;

public class SwerveModule implements Sendable {

  private final SparkMax turningSpark;
  private final SparkMax drivingSpark;
  private final SparkMaxConfig turningConfig;
  private final SparkMaxConfig drivingConfig;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;
  private AbsoluteEncoderChecker turningAbsoluteEncoderChecker = new AbsoluteEncoderChecker();
  private final SparkClosedLoopController drivingPIDController;
  private final SparkClosedLoopController turningPIDController;
  //    Radians offset for the module
  private double moduleOffset;
  private final boolean isInverted;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d(0));

  private double drivingEncoderPrevVelocity = 0;
  private double moduleAcceleration = 0;

  private double drivingEncoderPrevAcceleration = 0;
  private double moduleJerk = 0;

  private double lastNonSlippingWheelVelocity = 0;
  private double count = 0;

  private double desiredModuleAcceleration = 0;
  private double desiredModuleVelocity = 0;

  private Rotation2d previousMoudleAngle = Rotation2d.fromRadians(0);
  private double lastNonSlippingWheelAcceleration = 0;

  public SwerveModule(int drivingCanId, int turningCanId, double moduleOffset,boolean isInverted) {
    drivingSpark = new SparkMax(drivingCanId, MotorType.kBrushless);
    drivingConfig = new SparkMaxConfig();
    turningSpark = new SparkMax(turningCanId, MotorType.kBrushless);
    turningConfig = new SparkMaxConfig();
    this.moduleOffset = moduleOffset;
    this.isInverted = isInverted;

    SparkMaxUtils.initWithRetry(this::initDriveSpark, Constants.SPARK_INIT_RETRY_ATTEMPTS);
    SparkMaxUtils.initWithRetry(this::initTurnSpark, Constants.SPARK_INIT_RETRY_ATTEMPTS);

    drivingEncoder = drivingSpark.getEncoder();
    drivingPIDController = drivingSpark.getClosedLoopController();
    turningEncoder = turningSpark.getAbsoluteEncoder();
    turningPIDController = turningSpark.getClosedLoopController();

    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0.0);
    
  }

  public boolean initTurnSpark() {
    turningConfig
        .inverted(Constants.Swerve.Module.TURNING_ENCODER_INVERTED)
        .idleMode(Constants.Swerve.Module.TURNING_MOTOR_IDLE_MODE)
        .smartCurrentLimit(Constants.Swerve.Module.TURNING_MOTOR_CURRENT_LIMIT_AMPS);
    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.Swerve.Module.TURNING_K_P, Constants.Swerve.Module.TURNING_K_I, Constants.Swerve.Module.TURNING_K_D)
        .velocityFF(Constants.Swerve.Module.TURNING_K_FF)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS)
        .positionWrappingMaxInput(Constants.Swerve.Module.TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS)
        .outputRange(Constants.Swerve.Module.TURNING_MIN_OUTPUT, Constants.Swerve.Module.TURNING_MAX_OUTPUT);
        
    var configure_response = turningSpark.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return configure_response == REVLibError.kOk;
  }

  public boolean initDriveSpark() {
    drivingConfig
        .inverted(this.isInverted)
        .idleMode(Constants.Swerve.Module.DRIVING_MOTOR_IDLE_MODE)
        .smartCurrentLimit(Constants.Swerve.Module.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
    drivingConfig.encoder
        .positionConversionFactor(Constants.Swerve.Module.DRIVING_ENCODER_POSITION_FACTOR_METERS)
        .velocityConversionFactor(Constants.Swerve.Module.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND);
    drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(Constants.Swerve.Module.DRIVING_K_P, Constants.Swerve.Module.DRIVING_K_I, Constants.Swerve.Module.DRIVING_K_D)
        .velocityFF(Constants.Swerve.Module.DRIVING_K_FF)
        .positionWrappingEnabled(false)
        .outputRange(Constants.Swerve.Module.DRIVING_MIN_OUTPUT, Constants.Swerve.Module.DRIVING_MAX_OUTPUT);

    var configure_response = drivingSpark.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    return configure_response == REVLibError.kOk;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        drivingEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition() - moduleOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition() - moduleOffset));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    updateModuleKinematics();
    
    // count is a filter to deal with jittery jerk values
    if (isModuleJerkWithinThreshold()) {
      count++;
    } else {
      count = 0;
    }
    count = 2;
    // check if count is above 1 as simple filter
    if (count > 1) {
      // In this case, module acts like normal
      applyDesiredState(correctedDesiredState, desiredState);
    } else {
      // In this case, module locks to angle and acceleration
      applyCorrectedState(correctedDesiredState, desiredState);
    }

    SwerveModuleState optimizedState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(turningEncoder.getPosition()));
    this.desiredState = optimizedState;

    drivingPIDController.setReference(optimizedState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);
    turningPIDController.setReference(optimizedState.angle.getRadians(), SparkBase.ControlType.kPosition);
  }
    
  private void updateModuleKinematics() {
    moduleAcceleration = (getModuleVelocity() - drivingEncoderPrevVelocity) * 50;
    drivingEncoderPrevVelocity = getModuleVelocity();
    moduleJerk = (moduleAcceleration - drivingEncoderPrevAcceleration) * 50;
    drivingEncoderPrevAcceleration = moduleAcceleration;
  } 
    
  private boolean isModuleJerkWithinThreshold() {
    return getModuleJerk() < Constants.Swerve.Module.JERK_THRESHOLD;
  }
    
  private void applyDesiredState(SwerveModuleState correctedState, SwerveModuleState desiredState) { 
    correctedState.speedMetersPerSecond = desiredState.speedMetersPerSecond; 
    correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset)); 
    lastNonSlippingWheelAcceleration = moduleAcceleration; 
    previousMoudleAngle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset)); 
  } 
    
  private void applyCorrectedState(SwerveModuleState correctedState, SwerveModuleState desiredState) {
    //Module acceleration is locked here
    correctedState.speedMetersPerSecond = getModuleVelocity() + lastNonSlippingWheelAcceleration / 50;
    
    if (DriverStation.isAutonomous()) {
      // Disables module angle locking for traction control during auto. Otherwise auto becomes all screwy. 
      // The module anlge locking only matter for extremely sharp changes in desired direction. Which isn't a problem with pre-programmed paths.
      correctedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleOffset));
    } else {
      // Module angle is locked here
      correctedState.angle = previousMoudleAngle;
    }
  }

  public double getEncoderAbsPositionRad() {
    return turningEncoder.getPosition();
  }

  public double getDesiredModuleVelocity () {
    return desiredModuleVelocity;
  }

  public double getModuleVelocity(){
    return drivingEncoder.getVelocity();
  }

  public double getModuleAcceleration(){
    return moduleAcceleration;
  }

  public double getDesiredModuleAcceleration(){
    return desiredModuleAcceleration;
  }

  public double getModuleJerk(){
    return moduleJerk;
  }

  public double getLastNonSlippingWheelVelocity(){
    return lastNonSlippingWheelVelocity;
  }

  public double getModuleJerktoCurrent(){
    return Math.min(1000, Math.abs(getModuleJerk()) / (getCurrent()+2)); // +1 is for divide by zero error. Also, values shouldn't be too big at low speeds (Where current is 0)
  }

  public double getCurrent(){
    return drivingSpark.getOutputCurrent();
  }

  public void periodic() {
    //TODO: FIGURE OUT, DOES THIS DO ANYTHING OR ARE WE ACTING ON IT?
    turningAbsoluteEncoderChecker.addReading(turningEncoder.getPosition());
  }

  public void setModuleOffset(double offset){
    this.moduleOffset = offset;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Driving kP", drivingSpark.configAccessor.closedLoop::getP, drivingConfig.closedLoop::p);
    builder.addDoubleProperty("Driving kI", drivingSpark.configAccessor.closedLoop::getI, drivingConfig.closedLoop::i);
    builder.addDoubleProperty("Driving kD", drivingSpark.configAccessor.closedLoop::getD, drivingConfig.closedLoop::d);
    builder.addDoubleProperty("Driving kFF", drivingSpark.configAccessor.closedLoop::getFF, drivingConfig.closedLoop::velocityFF);
    builder.addDoubleProperty("Turning kP", turningSpark.configAccessor.closedLoop::getP, turningConfig.closedLoop::p);
    builder.addDoubleProperty("Turning kI", turningSpark.configAccessor.closedLoop::getI, turningConfig.closedLoop::i);
    builder.addDoubleProperty("Turning kD", turningSpark.configAccessor.closedLoop::getD, turningConfig.closedLoop::d);
    builder.addDoubleProperty("Turning kFF", turningSpark.configAccessor.closedLoop::getFF, turningConfig.closedLoop::velocityFF);
    builder.addDoubleProperty("Driving Vel (m/s)", drivingEncoder::getVelocity, null);
    builder.addDoubleProperty("Steering Pos (rad)", turningEncoder::getPosition, null);
    builder.addDoubleProperty("Desired Vel (m/s)", () -> desiredState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Desired Steer (rad)", () -> desiredState.angle.getRadians(), null);
    builder.addBooleanProperty("Turning encoder connected", turningAbsoluteEncoderChecker::encoderConnected, null);

  }

}