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
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leadSpark;
  private final SparkMax followerSpark;

  private final RelativeEncoder leadEncoder;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  private final PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI,
      ElevatorConstants.kD);

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetHeightPosition = 0.0;
  private double currentHeightPosition = 0.0;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("elevatorP", ElevatorConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("elevatorI", ElevatorConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("elevatorD", ElevatorConstants.kD);

  public ElevatorIOSpark() {
    // lead set up
    leadSpark = new SparkMax(ElevatorConstants.leadCanId, MotorType.kBrushless);
    leadEncoder = leadSpark.getEncoder();
    var leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .voltageCompensation(12.0)
        .inverted(true);

    tryUntilOk(
        leadSpark,
        5,
        () -> leadSpark.configure(
            leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(leadSpark, 5, () -> leadEncoder.setPosition(0.0));

    // follower set up
    followerSpark = new SparkMax(ElevatorConstants.followerCanId, MotorType.kBrushless);
    var followerEncoder = followerSpark.getEncoder();
    var followerConfig = new SparkMaxConfig();
    followerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ElevatorConstants.currentLimit)
        .follow(ElevatorConstants.leadCanId, ElevatorConstants.followerInverted);

    tryUntilOk(
        followerSpark,
        5,
        () -> followerSpark.configure(
            followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(followerSpark, 5, () -> followerEncoder.setPosition(0.0));
  }

  private void updateEncoderValues(ElevatorIOInputs inputs) {
    var leadEncoderValue = -leadEncoder.getPosition();
    var followerEncoderValue = -followerSpark.getEncoder().getPosition();
    currentHeightPosition = leadEncoderValue;

    inputs.leadEncoderPosition = leadEncoderValue;
    inputs.followerEncoderPosition = followerEncoderValue;
  }

  private void updateLimitSwitches(ElevatorIOInputs inputs) {
    if (leadSpark.getReverseLimitSwitch().isPressed()) {
      inputs.homed = true;
      inputs.isAtBottom = true;
      inputs.currentHeightPosition = 0;
      leadEncoder.setPosition(0);
    } else {
      inputs.isAtBottom = false;
    }
  }
  
  private double calculateOutput(ElevatorIOInputs inputs) {
    if (!inputs.leadSparkConnected)
      return 0.0;

    double output = 0;
    if (closedLoop) {
      if (inputs.homed) {
        output = controller.calculate(currentHeightPosition, targetHeightPosition);
      }
    } else {
      output = this.openLoopVoltage;
    }
    return output;
  }

  private void updateLogging(double output) {
    Logger.recordOutput("Elevator/CurrentHeightInches", currentHeightPosition);
    Logger.recordOutput("Elevator/ControlEffort", output);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sparkStickyFault = false;
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);
    inputs.followerSparkConnected = followerConnectedDebounce.calculate(!sparkStickyFault);

    controller.setPID(p.get(), i.get(), d.get());

    updateEncoderValues(inputs);
    updateLimitSwitches(inputs);

    double output = calculateOutput(inputs);
    leadSpark.setVoltage(output);

    inputs.currentHeightPosition = currentHeightPosition;
    inputs.targetHeightPosition = targetHeightPosition;

    updateLogging(output);
  }

  @Override
  public void setPercent(double percent) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    setVoltage(clamped * 12);
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    var clamped = MathUtil.clamp(voltage, -12, 12);
    openLoopVoltage = clamped;
  }

  @Override
  public void setTargetHeight(double targetHeightPosition) {
    closedLoop = true;
    this.targetHeightPosition = targetHeightPosition;
  }
}
