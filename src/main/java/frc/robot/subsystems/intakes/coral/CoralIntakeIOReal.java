package frc.robot.subsystems.intakes.coral;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralIntakeIOReal implements CoralIntakeIO {
  public final SparkMax spark1;
  public final SparkMax spark2;

  private final Debouncer spark1ConnectedDebounce = new Debouncer(0.5);
  private final Debouncer spark2ConnectedDebounce = new Debouncer(0.5);

  private final Timer currentSpikeTimer = new Timer();
  private boolean hasGamePiece = false;
  private double laserDistance = 0.0;

  public CoralIntakeIOReal() {
    spark1 = new SparkMax(CoralIntakeConstants.spark1CanId, MotorType.kBrushed);

    var config1 = new SparkMaxConfig();
    config1
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralIntakeConstants.currentLimit)
        .inverted(CoralIntakeConstants.spark1Inverted);

    tryUntilOk(
        spark1,
        5,
        () ->
            spark1.configure(
                config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    spark2 = new SparkMax(CoralIntakeConstants.spark2CanId, MotorType.kBrushed);

    var config2 = new SparkMaxConfig();
    config2
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralIntakeConstants.currentLimit)
        .inverted(CoralIntakeConstants.spark2Inverted);

    tryUntilOk(
        spark2,
        5,
        () ->
            spark2.configure(
                config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  private boolean detectGamePiece() {
    double totalCurrent = spark1.getOutputCurrent() + spark2.getOutputCurrent();

    if (totalCurrent >= CoralIntakeConstants.capturedCurrentOutput) {
      if (!currentSpikeTimer.isRunning()) {
        currentSpikeTimer.restart();
      }
      if (currentSpikeTimer.hasElapsed(0.25)) {
        return true;
      }
    } else {
      currentSpikeTimer.stop();
    }
    return false;
  }

  @Override
  public void updateInputs(CoralIntakeIOInputs inputs) {
    sparkStickyFault = false;
    inputs.spark1SparkConnected = spark1ConnectedDebounce.calculate(!sparkStickyFault);
    inputs.spark2SparkConnected = spark2ConnectedDebounce.calculate(!sparkStickyFault);

    inputs.spark1OutputCurrent = spark1.getOutputCurrent();
    inputs.spark2OutputCurrent = spark2.getOutputCurrent();

    inputs.spark1SpeedPercent = spark1.get();
    inputs.spark2SpeedPercent = spark2.get();

    inputs.spark1Engaged = spark1.get() != 0;
    inputs.spark2Engaged = spark2.get() != 0;

    inputs.hasGamePiece = detectGamePiece();

    inputs.totalOutputCurrent = spark1.getOutputCurrent() + spark2.getOutputCurrent();

    SmartDashboard.putNumber("CoralIntake/Spark1Speed", inputs.spark1SpeedPercent);
    SmartDashboard.putNumber("CoralIntake/Spark1Current", inputs.spark1OutputCurrent);

    SmartDashboard.putNumber("CoralIntake/Spark2Speed", inputs.spark2SpeedPercent);
    SmartDashboard.putNumber("CoralIntake/Spark2Current", inputs.spark2OutputCurrent);
  }

  @Override
  public void setPercent(double percent) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    spark1.set(clamped);
    spark2.set(-clamped);
  }
}
