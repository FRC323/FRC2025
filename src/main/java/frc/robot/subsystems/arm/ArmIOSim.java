package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim armSim;

  private final ProfiledPIDController controller;
  private final ArmFeedforward feedforward;
  private final TrapezoidProfile.Constraints constraints;

  private final Mechanism2d armVisualization;
  private final MechanismRoot2d armPivot;
  private final MechanismLigament2d armLigamentRight;
  private final MechanismLigament2d armLigamentLeft;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("armP", ArmConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("armI", ArmConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("armD", ArmConstants.kD);

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetPostionRadians = 0.0;

  public ArmIOSim() {
    LinearSystem<N2, N1, N2> armSystem =
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.1, ArmConstants.gearRatio);

    armSim =
        new SingleJointedArmSim(
            armSystem,
            DCMotor.getNEO(1).withReduction(ArmConstants.gearRatio),
            ArmConstants.gearRatio,
            ArmConstants.ArmLengthMeters,
            Units.degreesToRadians(-360),
            Units.degreesToRadians(360),
            true,
            Units.degreesToRadians(-90));
    constraints =
        new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration);
    controller = new ProfiledPIDController(p.get(), i.get(), d.get(), constraints);
    feedforward =
        new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

    armVisualization = new Mechanism2d(6, 2);
    armPivot = armVisualization.getRoot("ArmPivot", 3, 1);

    armLigamentRight =
        armPivot.append(
            new MechanismLigament2d("ArmRight", 0.75, 0, 6, new Color8Bit(255, 255, 255)));

    armLigamentLeft =
        armPivot.append(new MechanismLigament2d("ArmLeft", 0.75, 0, 6, new Color8Bit(0, 255, 0)));

    armLigamentRight.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    armLigamentLeft.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    SmartDashboard.putData("Arm Simulation", armVisualization);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());
    inputs.leadSparkConnected = true;

    if (!RobotState.isEnabled()) {
      armSim.setInput(0.0);
      armSim.setState(armSim.getAngleRads(), 0.0);
    }

    armSim.update(ArmConstants.kDt);

    double output = 0.0;
    if (closedLoop) {
      output = controller.calculate(armSim.getAngleRads(), inputs.targetPositionRadians);
      var ff = feedforward.calculate(inputs.targetPositionRadians, 0);
      output += ff;
      armSim.setInput(output);

      Logger.recordOutput("Arm/Feedforward", ff);
    } else {
      output = this.openLoopVoltage;

      armSim.setInput(output);
    }

    // same for sim - do i care?
    inputs.currentAbsolutePositionRadians = armSim.getAngleRads();
    inputs.currentRelativePositionRadians = armSim.getAngleRads();
    inputs.targetPositionRadians = this.targetPostionRadians;

    if (reachedDesiredPosition(
        inputs.targetPositionRadians, inputs.currentAbsolutePositionRadians)) {
      inputs.reachedDesiredPosition = true;
      armSim.setInput(0);
    } else {
      inputs.reachedDesiredPosition = false;
    }

    double currentAngleDegrees = Units.radiansToDegrees(armSim.getAngleRads());
    armLigamentRight.setAngle(currentAngleDegrees);
    armLigamentLeft.setAngle(currentAngleDegrees + 180);

    SmartDashboard.putData("Arm Simulation", armVisualization);

    Logger.recordOutput("Arm/ControlEffort", output);
  }

  private boolean reachedDesiredPosition(double current, double target) {
    return isWithinTolerance(target, current, 0.02);
  }

  private boolean isWithinTolerance(double current, double target, double tolerance) {
    return Math.abs(target - current) <= tolerance;
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
  public void setAngleRadians(double targetAngleRadians) {
    this.closedLoop = true;
    this.targetPostionRadians = targetAngleRadians;
  }
}
