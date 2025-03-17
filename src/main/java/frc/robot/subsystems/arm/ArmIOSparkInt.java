package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmIOSparkInt implements ArmIO {
  private final SparkMax leadSpark;

  private final AbsoluteEncoder leadAbsoluteEncoder;
  private final RelativeEncoder leadRelativeEncoder;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);

  public final SparkClosedLoopController controller;
  public final ArmFeedforward feedForward;

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetPosition = 0.0;
  private boolean armCanMove = false;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("armP", ArmConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("armI", ArmConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("armD", ArmConstants.kD);

  public ArmIOSparkInt() {
    leadSpark = new SparkMax(ArmConstants.leadCanId, MotorType.kBrushless);

    controller = leadSpark.getClosedLoopController();
    feedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);

    leadAbsoluteEncoder = leadSpark.getAbsoluteEncoder();
    leadRelativeEncoder = leadSpark.getEncoder();

    var leadConfig = new SparkMaxConfig();
    leadConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ArmConstants.currentLimit)
        .inverted(ArmConstants.leadInverted);
    leadConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ArmConstants.kP)
        .i(ArmConstants.kI)
        .d(ArmConstants.kD)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(
            1.0 / 473,
            ClosedLoopSlot
                .kSlot1) // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    tryUntilOk(
        leadSpark,
        5,
        () ->
            leadSpark.configure(
                leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // absolute enc has rollover point, if the arm starts on the wrong side, shut
    // down
    // we want the arm to start >= 0, not < 0, as it will cause rotation issues
    this.armCanMove = leadAbsoluteEncoder.getPosition() >= 0;
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // controller.setPID(p.get(), i.get(), d.get());

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

    REVLibError error = null;
    if (inputs.leadSparkConnected) {
      if (armCanMove) {
        if (leadAbsoluteEncoder.getPosition() >= 0 && leadAbsoluteEncoder.getPosition() <= .80) {
          if (closedLoop) {
            error = controller.setReference(inputs.currentAbsolutePosition, ControlType.kPosition);
          } else {
            error =
                controller.setReference(
                    this.openLoopVoltage, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
          }
        }
      }
    }
    Logger.recordOutput("Arm/ControlStatus", error.toString());
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
