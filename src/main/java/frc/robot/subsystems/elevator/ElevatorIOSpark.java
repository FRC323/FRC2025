package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leadSpark;
  private final SparkMax followerSpark;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);
  private final Debouncer bottomLimitSwitchDebounce = new Debouncer(0.2);

  private final RelativeEncoder leadEncoder;
  private final RelativeEncoder followerEncoder;

  private final PIDController controller =
      new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetPosition = 0.0;
  private ElevatorPosition opsCommandedLevel;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("elevatorP", ElevatorConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("elevatorI", ElevatorConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("elevatorD", ElevatorConstants.kD);

  public ElevatorIOSpark() {
    opsCommandedLevel = ElevatorPosition.Home;
    // lead set up
    leadSpark = new SparkMax(ElevatorConstants.leadCanId, MotorType.kBrushless);
    leadEncoder = leadSpark.getEncoder();
    var leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .inverted(ElevatorConstants.leadInverted);

    tryUntilOk(
        leadSpark,
        5,
        () ->
            leadSpark.configure(
                leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(leadSpark, 5, () -> leadEncoder.setPosition(0.0));

    // follower set up
    followerSpark = new SparkMax(ElevatorConstants.followerCanId, MotorType.kBrushless);
    followerEncoder = followerSpark.getEncoder();
    var followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .follow(ElevatorConstants.leadCanId, ElevatorConstants.followerInverted);

    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(followerSpark, 5, () -> followerEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());

    sparkStickyFault = false;
    ifOk(leadSpark, leadEncoder::getPosition, (value) -> inputs.leadEncoderPosition = value);
    ifOk(leadSpark, leadEncoder::getVelocity, (value) -> inputs.leadEncoderVelocity = value);
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        followerSpark,
        followerEncoder::getPosition,
        (value) -> inputs.followerEncoderPosition = -value);
    ifOk(
        followerSpark,
        leadEncoder::getVelocity,
        (value) -> inputs.followerEncoderVelocity = -value);
    inputs.followerSparkConnected = followerConnectedDebounce.calculate(!sparkStickyFault);

    var bottomLimitSwitchPressed =
        bottomLimitSwitchDebounce.calculate(leadSpark.getReverseLimitSwitch().isPressed());
    if (bottomLimitSwitchPressed) {
      inputs.homed = true;
      inputs.isAtBottom = true;
      leadEncoder.setPosition(0);
    } else {
      inputs.isAtBottom = false;
    }

    double output = 0;
    if (closedLoop) {
      if (inputs.homed) {
        double pidOutput = controller.calculate(inputs.leadEncoderPosition, this.targetPosition);
        output = pidOutput;
        leadSpark.set(output);
      }
    } else {
      output = this.openLoopVoltage;
      leadSpark.setVoltage(output);
    }

    SmartDashboard.putNumber("Elevator.Position", inputs.leadEncoderPosition);
    SmartDashboard.putNumber("Elevator.Velocity", inputs.leadEncoderVelocity);
    SmartDashboard.putBoolean("Elevator.AtBottom", inputs.isAtBottom);
    SmartDashboard.putNumber("Elevator.TargetPosition", targetPosition);
  }

  @Override
  public void setPercent(double percent) {
    closedLoop = false;
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    setVoltage(clamped * 12);
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    var clamped = MathUtil.clamp(voltage, -12, 12);
    leadSpark.setVoltage(clamped);
  }

  @Override
  public void setPosition(double targetPosition) {
    this.closedLoop = true;
    this.targetPosition = targetPosition;
  }
}
