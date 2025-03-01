package frc.robot.subsystems.climber;

import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.arm.ArmConstants;

public class ClimberIOSpark implements ClimberIO {
  public final SparkMax leadSpark;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);

  public ClimberIOSpark() {
    leadSpark = new SparkMax(ArmConstants.leadCanId, MotorType.kBrushless);

    var leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ArmConstants.currentLimit)
        .inverted(ArmConstants.leadInverted);

    tryUntilOk(
        leadSpark,
        5,
        () ->
            leadSpark.configure(
                leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sparkStickyFault = false;
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);
  }
}
