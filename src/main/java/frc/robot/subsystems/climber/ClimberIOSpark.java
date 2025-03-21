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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.climber.Climber.ClimberPosition;

public class ClimberIOSpark implements ClimberIO {
  public final SparkMax leadSpark;
  public final RelativeEncoder leadEncoder;

  private final PIDController controller = new PIDController(ClimberConstants.kP, 0.0, 0.0);

  public ClimberPosition targetPosition = ClimberPosition.Stowed;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);

  public double openLoopVoltage = 0.0;
  public boolean closedLoop = false;

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
    double currentPosition = leadEncoder.getPosition();

    double output = 0.0;
    if (closedLoop) {
      var rawPosition = getRawPosition(targetPosition);
      output = controller.calculate(leadEncoder.getPosition(), rawPosition);
      leadSpark.set(output);
    } else {
      if (this.targetPosition == ClimberPosition.Climb
          && currentPosition < ClimberConstants.ClimbedPosition) {
        output = this.openLoopVoltage;
        leadSpark.setVoltage(output);
      } else if (this.targetPosition == ClimberPosition.Deploy
          && currentPosition < ClimberConstants.DeployedPosition) {
        output = this.openLoopVoltage;
        leadSpark.setVoltage(output);
      } else if (this.targetPosition == ClimberPosition.Stowed
          && currentPosition > ClimberConstants.StowedPosition) {
        output = -this.openLoopVoltage;
        leadSpark.setVoltage(output);
      } else {
        leadSpark.setVoltage(0);
      }
    }

    SmartDashboard.putNumber("Climber/ControlOutput", this.openLoopVoltage);
    SmartDashboard.putNumber("Climber/CurrentPosition", inputs.leadEncoderPosition);
    SmartDashboard.putNumber("Climber/TargetPosition", getRawPosition(targetPosition));
  }

  @Override
  public void resetZero() {
    leadEncoder.setPosition(0.0);
  }

  @Override
  public double getTargetPosition() {
    return getRawPosition(targetPosition);
  }

  @Override
  public double getCurrentPosition() {
    return leadEncoder.getPosition();
  }

  @Override
  public void setPercent(double percent, ClimberPosition position) {
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    setVoltage(clamped * 12, position);
  }

  @Override
  public void setVoltage(double voltage, ClimberPosition position) {
    var clamped = MathUtil.clamp(voltage, -12, 12);
    this.targetPosition = position;
    this.openLoopVoltage = clamped;
    this.closedLoop = false;
  }

  public void setPosition(ClimberPosition position) {
    this.targetPosition = position;
    // var rawPosition = getRawPosition(position);
    // MathUtil.clamp(rawPosition, ClimberConstants.StowedPosition,
    // ClimberConstants.ClimbedPosition);
    this.closedLoop = true;
  }

  private double getRawPosition(ClimberPosition position) {
    if (position == ClimberPosition.Deploy) {
      return ClimberConstants.DeployedPosition;
    } else if (position == ClimberPosition.Climb) {
      return ClimberConstants.ClimbedPosition;
    } else {
      return ClimberConstants.StowedPosition;
    }
  }

  @Override
  public void stop() {
    this.openLoopVoltage = 0.0;
    leadSpark.setVoltage(0);
  }
}
