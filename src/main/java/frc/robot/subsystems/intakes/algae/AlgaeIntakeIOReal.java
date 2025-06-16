package frc.robot.subsystems.intakes.algae;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;

public class AlgaeIntakeIOReal implements AlgaeIntakeIO {
  public final SparkMax spark1;
  public final SparkMax spark2;

  private final Debouncer spark1ConnectedDebounce = new Debouncer(0.5);
  private final Debouncer spark2ConnectedDebounce = new Debouncer(0.5);

  public AlgaeIntakeIOReal() {
    spark1 = new SparkMax(AlgaeIntakeConstants.spark1CanId, MotorType.kBrushed);

    var config1 = new SparkMaxConfig();
    config1
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeIntakeConstants.currentLimit)
        .inverted(AlgaeIntakeConstants.spark1Inverted);

    tryUntilOk(
        spark1,
        5,
        () ->
            spark1.configure(
                config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    spark2 = new SparkMax(AlgaeIntakeConstants.spark2CanId, MotorType.kBrushed);

    var config2 = new SparkMaxConfig();
    config2
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeIntakeConstants.currentLimit)
        .voltageCompensation(12.0)
        .inverted(AlgaeIntakeConstants.spark2Inverted);

    tryUntilOk(
        spark2,
        5,
        () ->
            spark2.configure(
                config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    sparkStickyFault = false;
    inputs.spark1SparkConnected = spark1ConnectedDebounce.calculate(!sparkStickyFault);
    inputs.spark2SparkConnected = spark2ConnectedDebounce.calculate(!sparkStickyFault);

    inputs.spark1OutputCurrent = spark1.getOutputCurrent();
    inputs.spark2OutputCurrent = spark2.getOutputCurrent();

    inputs.spark1SpeedPercent = spark1.get();
    inputs.spark2SpeedPercent = spark2.get();

    inputs.spark1Engaged = spark1.get() != 0;
    inputs.spark2Engaged = spark2.get() != 0;

    Logger.recordOutput("AlgaeIntake/Spark1/Connected", inputs.spark1SparkConnected);
    Logger.recordOutput("AlgaeIntake/Spark1/Engaged", inputs.spark1Engaged);
    Logger.recordOutput("AlgaeIntake/Spark1/SpeedPercent", inputs.spark1SpeedPercent);
    Logger.recordOutput("AlgaeIntake/Spark1/Current", inputs.spark1OutputCurrent);

    Logger.recordOutput("AlgaeIntake/Spark2/Connected", inputs.spark2SparkConnected);
    Logger.recordOutput("AlgaeIntake/Spark2/Engaged", inputs.spark2Engaged);
    Logger.recordOutput("AlgaeIntake/Spark2/SpeedPercent", inputs.spark2SpeedPercent);
    Logger.recordOutput("AlgaeIntake/Spark2/Current", inputs.spark2OutputCurrent);
  }

  @Override
  public void setPercent(double percent) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    spark1.set(clamped);
    spark2.set(clamped);
  }
}
