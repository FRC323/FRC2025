package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.arm.ArmConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSpark implements ClimberIO {
  public final SparkMax leadSpark;
  public final RelativeEncoder leadEncoder;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);

  public double openLoopVoltage = 0.0;

  public ClimberIOSpark() {
    leadSpark = new SparkMax(ClimberConstants.leadCanId, MotorType.kBrushless);
    leadEncoder = leadSpark.getEncoder();

    var leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConstants.currentLimit)
        .inverted(ArmConstants.leadInverted);

    tryUntilOk(
        leadSpark,
        5,
        () ->
            leadSpark.configure(
                leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(leadSpark, 5, () -> leadEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(leadSpark, leadEncoder::getPosition, (value) -> inputs.leadEncoderPosition = value);
    ifOk(leadSpark, leadEncoder::getVelocity, (value) -> inputs.leadEncoderVelocity = value);
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);

    inputs.CurrentOutput = leadSpark.getOutputCurrent();

    Logger.recordOutput("Climber/ControlOutput", this.openLoopVoltage);
  }

  @Override
  public double getPosition() {
    return leadEncoder.getPosition();
  }

  @Override
  public void setPercent(double percent) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    setVoltage(clamped * 12);
  }

  @Override
  public void setVoltage(double voltage) {
    var clamped = MathUtil.clamp(voltage, -12, 12);
    leadSpark.setVoltage(clamped);
  }
}
