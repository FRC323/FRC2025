package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmIOSpark implements ArmIO {
  private final SparkMax leadSpark;

  private final AbsoluteEncoder leadAbsoluteEncoder;
  private final RelativeEncoder leadRelativeEncoder;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, constraints);

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double armOffset = 0.0;
  private double targetPositionRadians = 0.0;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("armP", ArmConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("armI", ArmConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("armD", ArmConstants.kD);

  public ArmIOSpark() {
    leadSpark = new SparkMax(ArmConstants.leadCanId, MotorType.kBrushless);
    leadAbsoluteEncoder = leadSpark.getAbsoluteEncoder();
    leadRelativeEncoder = leadSpark.getEncoder();

    var leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ArmConstants.currentLimit)
        .voltageCompensation(12.0)
        .inverted(ArmConstants.leadInverted);

    tryUntilOk(
        leadSpark,
        5,
        () ->
            leadSpark.configure(
                leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    armOffset = Preferences.getDouble(ArmConstants.ArmOffsetKey, 0.0);
    double currentAbsolutePosition = leadAbsoluteEncoder.getPosition();
    double initialPosition = currentAbsolutePosition - armOffset;

    tryUntilOk(leadSpark, 5, () -> leadRelativeEncoder.setPosition(initialPosition));
  }

  private double calculateOutput(ArmIOInputs inputs) {
    if (!inputs.leadSparkConnected) return 0.0;

    double output = 0;
    if (closedLoop) {
      output = controller.calculate(inputs.currentRelativePositionRadians, targetPositionRadians);
    } else {
      output = this.openLoopVoltage;
    }
    return output;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());

    sparkStickyFault = false;
    ifOk(
        leadSpark,
        leadRelativeEncoder::getPosition,
        (value) -> inputs.currentRelativePositionRadians = Units.rotationsToRadians(value));
    ifOk(
        leadSpark,
        leadAbsoluteEncoder::getPosition,
        (value) -> inputs.currentAbsolutePositionRadians = Units.rotationsToRadians(value));
    ifOk(leadSpark, leadRelativeEncoder::getVelocity, (value) -> inputs.currentVelocity = value);
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);
    inputs.armOffset = armOffset;

    double encoderDrift =
        Math.abs(
            inputs.currentAbsolutePositionRadians
                - (inputs.currentRelativePositionRadians + armOffset));
    Logger.recordOutput("Arm/EncoderDrift", encoderDrift);
    Logger.recordOutput(
        "Arm/EncoderDriftExceedsLimit", encoderDrift > ArmConstants.maxEncoderDrift);

    double output = calculateOutput(inputs);
    leadSpark.set(output);

    Logger.recordOutput(
        "Arm/CurrentAbsolutePositionRadians", inputs.currentAbsolutePositionRadians);
    Logger.recordOutput(
        "Arm/CurrentRelativePositionRadians", inputs.currentRelativePositionRadians);
    Logger.recordOutput("Arm/ControlEffort", output);
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
  public void setAngleRadians(double positionRadians) {
    targetPositionRadians =
        MathUtil.clamp(positionRadians, ArmConstants.minAngleRadians, ArmConstants.maxAngleRadians);
    closedLoop = true;
  }

  @Override
  public void storeOffset() {
    var encoder_position = leadAbsoluteEncoder.getPosition();
    var key = ArmConstants.ArmOffsetKey;
    Preferences.setDouble(key, encoder_position);
    System.out.println("Set Arm " + key + ": " + encoder_position);
  }
}
