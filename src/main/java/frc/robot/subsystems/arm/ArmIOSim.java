package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
  private final MechanismLigament2d armLigament;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("armP", ArmConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("armI", ArmConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("armD", ArmConstants.kD);

  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetPostionRadians = 0.0;

  public ArmIOSim() {
    LinearSystem<N2, N1, N2> armSystem =
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.1, 100);

    armSim =
        new SingleJointedArmSim(
            armSystem,
            DCMotor.getNEO(1).withReduction(100),
            100,
            0.5,
            Units.degreesToRadians(-180),
            Units.degreesToRadians(180),
            true,
            Units.degreesToRadians(90));
    constraints =
        new TrapezoidProfile.Constraints(ArmConstants.maxVelocity, ArmConstants.maxAcceleration);
    controller = new ProfiledPIDController(p.get(), i.get(), d.get(), constraints);
    feedforward =
        new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

    armVisualization = new Mechanism2d(6, 2);
    armPivot = armVisualization.getRoot("ArmPivot", 3, 1);

    armLigament =
        armPivot.append(new MechanismLigament2d("Arm", 1.5, 0, 6, new Color8Bit(255, 0, 0)));
    armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));

    SmartDashboard.putData("Arm Simulation", armVisualization);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());

    armSim.update(.02);

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

    inputs.currentPositionRadians = armSim.getAngleRads();
    inputs.targetPositionRadians = this.targetPostionRadians;

    if (reachedDesiredPosition(inputs.targetPositionRadians, inputs.currentPositionRadians)) {
      inputs.reachedDesiredPosition = true;
      armSim.setInput(0);
    } else {
      inputs.reachedDesiredPosition = false;
    }

    armLigament.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
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
  public void setAngleRadians(double targetAngleRadians) {
    this.closedLoop = true;
    this.targetPostionRadians = targetAngleRadians;
  }

  @Override
  public void setOpenLoop(double output) {
    this.closedLoop = false;
    this.openLoopVoltage = output;
  }
}
