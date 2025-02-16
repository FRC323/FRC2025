package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim;
  private final PIDController controller;
  private final ElevatorFeedforward feedforward;

  // Mechanism2d visualization
  private final Mechanism2d mechanism2d;
  private final MechanismRoot2d elevatorRoot;
  private final MechanismLigament2d elevatorCarriage;

  private final DIOSim bottomLimitSwitch;
  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetHeightInches = 0.0;
  private double currentHeightInches = 0.0;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("elevatorP", ElevatorConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("elevatorI", ElevatorConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("elevatorD", ElevatorConstants.kD);

  public ElevatorIOSim() {
    bottomLimitSwitch = new DIOSim(ElevatorConstants.bottomLimitSwitchChannel);

    LinearSystem<N2, N1, N2> elevatorSystem =
        LinearSystemId.createElevatorSystem(
            DCMotor.getNEO(2).withReduction(ElevatorConstants.gearRatio),
            ElevatorConstants.carriageMass,
            ElevatorConstants.drumRadius,
            ElevatorConstants.gearRatio);

    elevatorSim =
        new ElevatorSim(
            elevatorSystem,
            DCMotor.getNEO(2).withReduction(ElevatorConstants.gearRatio),
            Units.inchesToMeters(ElevatorConstants.minElevatorHeight),
            Units.inchesToMeters(ElevatorConstants.maxElevatorHeight),
            true,
            0);

    TrapezoidProfile.Constraints constraints =
        new TrapezoidProfile.Constraints(
            ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
    controller = new PIDController(p.get(), i.get(), d.get());
    feedforward =
        new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    // Setup Mechanism2d visualization
    mechanism2d = new Mechanism2d(40, ElevatorConstants.maxElevatorHeight);
    elevatorRoot = mechanism2d.getRoot("Elevator", 20, 0);
    elevatorCarriage = elevatorRoot.append(new MechanismLigament2d("Carriage", 0, 90));

    SmartDashboard.putData("Elevator Simulation", mechanism2d);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());
    inputs.leadSparkConnected = true;
    inputs.followerSparkConnected = true;

    elevatorSim.update(ElevatorConstants.kDt);

    bottomLimitSwitch.setValue(this.currentHeightInches <= 0);

    if (bottomLimitSwitch.getValue()) {
      elevatorSim.setInput(0);
      elevatorSim.setState(0, 0);
      inputs.homed = true;
      inputs.isAtBottom = true;
      inputs.currentHeightPosition = 0;
    } else {
      inputs.isAtBottom = false;
    }

    double output = 0;
    if (closedLoop) {
      if (inputs.homed) {
        double pidOutput =
            controller.calculate(
                Units.metersToInches(elevatorSim.getPositionMeters()), inputs.targetHeightPosition);
        // double ffOutput =
        // feedforward.calculate(profiledPidController.getSetpoint().velocity);
        output = pidOutput; // + ffOutput;

        elevatorSim.setInput(output);
      }
    } else {
      output = this.openLoopVoltage;
      // System.out.println("ElevatorIOSim. OpenLoop effort: {" + output + "}");
      elevatorSim.setInput(output);
    }

    currentHeightInches = Units.metersToInches(elevatorSim.getPositionMeters());

    inputs.currentHeightPosition = this.currentHeightInches;
    inputs.targetHeightPosition = this.targetHeightInches;

    elevatorCarriage.setLength(inputs.currentHeightPosition);

    Logger.recordOutput("Elevator/ControlEffort", output);
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
  public void setTargetHeight(double targetHeightInches) {
    closedLoop = true;
    this.targetHeightInches = targetHeightInches;
  }
}
