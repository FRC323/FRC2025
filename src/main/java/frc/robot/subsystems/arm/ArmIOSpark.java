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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmIOSpark implements ArmIO {
  private final SparkMax leadSpark;

  private final AbsoluteEncoder leadAbsoluteEncoder;
  private final RelativeEncoder leadRelativeEncoder;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);

  // private final TrapezoidProfile.Constraints constraints =
  // new TrapezoidProfile.Constraints(ArmConstants.maxVelocity,
  // ArmConstants.maxAcceleration);
  final PIDController controller =
      new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  // private final ArmFeedforward feedForward =
  // new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetPosition = 0.0;
  // private boolean armCanMove = false;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("armP", ArmConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("armI", ArmConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("armD", ArmConstants.kD);

  public ArmIOSpark() {
    leadSpark = new SparkMax(ArmConstants.leadCanId, MotorType.kBrushless);
    leadAbsoluteEncoder = leadSpark.getAbsoluteEncoder();
    leadRelativeEncoder = leadSpark.getEncoder();

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

    // absolute enc has rollover point, if the arm starts on the wrong side, shut down
    // we want the arm to start >= 0, not < 0, as it will cause rotation issues
    // this.armCanMove = leadAbsoluteEncoder.getPosition() >= 0;
  }
  // 44.52
  private double calculateOutput(ArmIOInputs inputs) {
    if (!inputs.leadSparkConnected) return 0.0;
    // if (!armCanMove) {
    //   System.out.println("MOVE ARM TO >= 0!!!!!!!!!");
    //   return 0.0;
    // }

    if (leadAbsoluteEncoder.getPosition() >= 0 && leadAbsoluteEncoder.getPosition() <= .80) {
      double output = 0;
      if (closedLoop) {
        output = controller.calculate(leadAbsoluteEncoder.getPosition(), targetPosition);
      } else {
        output = this.openLoopVoltage;
      }
      return output;
    }
    return 0.0;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());

    sparkStickyFault = false;
    ifOk(
        leadSpark,
        leadRelativeEncoder::getPosition,
        (value) -> inputs.currentRelativePosition = value);
    ifOk(
        leadSpark,
        leadAbsoluteEncoder::getPosition,
        (value) -> inputs.currentAbsolutePosition = value);
    ifOk(leadSpark, leadRelativeEncoder::getVelocity, (value) -> inputs.currentVelocity = value);
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);

    inputs.targetPosition = this.targetPosition;

    double output = calculateOutput(inputs);
    leadSpark.set(output);

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
  public void setAngleRadians(double position) {
    this.targetPosition =
        MathUtil.clamp(position, ArmConstants.minPosition, ArmConstants.maxPosition);
    closedLoop = true;
  }
}
