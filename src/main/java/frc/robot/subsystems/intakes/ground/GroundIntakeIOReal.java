package frc.robot.subsystems.intakes.ground;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
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

public class GroundIntakeIOReal implements GroundIntakeIO {
  private final SparkMax wristLeadSpark;
  private final Debouncer wristLeadConnectedDebounce = new Debouncer(0.5);

  private final SparkMax intakeLeadSpark;
  private final Debouncer intakeLeadConnectedDebounce = new Debouncer(0.5);

  private final AbsoluteEncoder wristAbsoluteEncoder;
  private final PIDController wristController =
      new PIDController(
          GroundIntakeConstants.wristP, GroundIntakeConstants.wristI, GroundIntakeConstants.wristD);

  private double wristTargetPosition = 0.0;
  private boolean wristClosedLoop = false;
  private double wristOpenLoopVoltage = 0.0;

  public GroundIntakeIOReal() {
    // wrist
    wristLeadSpark = new SparkMax(GroundIntakeConstants.wirstSparkCanId, MotorType.kBrushless);
    wristAbsoluteEncoder = wristLeadSpark.getAbsoluteEncoder();

    var wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(GroundIntakeConstants.wristCurrentLimit)
        .inverted(GroundIntakeConstants.wirstSparkInverted);

    tryUntilOk(
        wristLeadSpark,
        5,
        () ->
            wristLeadSpark.configure(
                wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // intake
    intakeLeadSpark = new SparkMax(GroundIntakeConstants.intakeSparkCanId, MotorType.kBrushless);

    var intakeConfig = new SparkMaxConfig();
    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(GroundIntakeConstants.intakeCurrentLimit)
        .inverted(GroundIntakeConstants.intakeSparkInverted);

    tryUntilOk(
        wristLeadSpark,
        5,
        () ->
            wristLeadSpark.configure(
                wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(GroundIntakeIOInputs inputs) {
    // wrist
    sparkStickyFault = false;
    ifOk(
        wristLeadSpark,
        wristLeadSpark::getOutputCurrent,
        (value) -> inputs.wristOutputCurrent = value);
    inputs.wristSparkConnected = wristLeadConnectedDebounce.calculate(!sparkStickyFault);

    // intake
    sparkStickyFault = false;
    ifOk(
        intakeLeadSpark,
        intakeLeadSpark::getOutputCurrent,
        (value) -> inputs.intakeOutputCurrent = value);
    inputs.intakeSparkConnected = intakeLeadConnectedDebounce.calculate(!sparkStickyFault);

    inputs.wristCurrentPosition = wristAbsoluteEncoder.getPosition();
    inputs.wristTargetPosition = this.wristTargetPosition;

    double output = 0;
    if (wristClosedLoop) {
      output =
          wristController.calculate(wristAbsoluteEncoder.getPosition(), this.wristTargetPosition);
      wristLeadSpark.set(output);
    } else {
      inputs.wristTargetPosition = wristOpenLoopVoltage;
      wristLeadSpark.setVoltage(output);
    }

    SmartDashboard.putNumber("GroundIntake/CurrentWristPosition", inputs.wristCurrentPosition);
    SmartDashboard.putNumber("GroundIntake/TargetWristPosition", inputs.wristTargetPosition);
    SmartDashboard.putNumber("GroundIntake/WristOutput", output);
  }

  @Override
  public double getWristPosition() {
    return wristAbsoluteEncoder.getPosition();
  }

  @Override
  public void setWristPercent(double percent) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    setWristVoltage(clamped * 12);
  }

  @Override
  public void setWristVoltage(double voltage) {
    wristClosedLoop = false;
    var clamped = MathUtil.clamp(voltage, -12, 12);
    wristOpenLoopVoltage = clamped;
  }

  @Override
  public void setWristPosition(double position) {
    this.wristTargetPosition =
        MathUtil.clamp(
            position,
            GroundIntakeConstants.wristMinPosition,
            GroundIntakeConstants.wristMaxPosition);
    wristClosedLoop = true;
  }

  @Override
  public void setIntakePercent(double percent) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    intakeLeadSpark.set(clamped);
  }
}
