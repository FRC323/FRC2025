package frc.robot.subsystems.arm;

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
  private double targetPositionRadians = 0.0;

  private boolean useAbsoluteEncoder = true;
  private double initialPosition = 0.0;

  private double currentAbsolutePositionRadians = 0.0;
  private double currentRelativePositionRadians = 0.0;

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
    double absolutePosition =
        Preferences.getDouble(ArmConstants.ArmOffsetKey, leadAbsoluteEncoder.getPosition());

    if (Math.abs(leadAbsoluteEncoder.getPosition() - absolutePosition) > .05) {
      Logger.recordOutput("Arm/EncoderMismatch", true);
    }
    tryUntilOk(leadSpark, 5, () -> leadRelativeEncoder.setPosition(absolutePosition));
  }

  private void updateEncoderValues(ArmIOInputs inputs) {
    currentRelativePositionRadians = Units.rotationsToRadians(leadRelativeEncoder.getPosition());
    currentAbsolutePositionRadians = Units.rotationsToRadians(leadAbsoluteEncoder.getPosition());

    inputs.currentAbsolutePositionRadians = currentAbsolutePositionRadians;
    inputs.currentRelativePositionRadians = currentRelativePositionRadians;
  }

  private double calculateOutput(ArmIOInputs inputs) {
    if (!inputs.leadSparkConnected) return 0.0;

    double output = 0;
    if (closedLoop) {
      output = controller.calculate(currentRelativePositionRadians, targetPositionRadians);
    } else {
      output = this.openLoopVoltage;
    }
    return output;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);

    controller.setPID(p.get(), i.get(), d.get());

    if (useAbsoluteEncoder) {
      initialPosition = Units.rotationsToRadians(leadAbsoluteEncoder.getPosition());
      currentAbsolutePositionRadians = initialPosition;
      currentRelativePositionRadians = initialPosition;
      useAbsoluteEncoder = false;
    } else {
      updateEncoderValues(inputs);
    }

    double output = calculateOutput(inputs);
    leadSpark.set(output);

    inputs.currentAbsolutePositionRadians = currentAbsolutePositionRadians;
    inputs.currentRelativePositionRadians = currentRelativePositionRadians;

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
    targetPositionRadians = positionRadians;
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
