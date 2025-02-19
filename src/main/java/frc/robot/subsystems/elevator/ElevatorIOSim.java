package frc.robot.subsystems.elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
  private final SparkMaxSim leadSpark;
  private final SparkMaxSim followerSpark;
  private final SparkRelativeEncoderSim leadEncoder;
  private final DCMotor motor = DCMotor.getNEO(1).withReduction(ElevatorConstants.gearRatio);
  // private final ElevatorFeedforward feedforward;

  // Mechanism2d visualization
  private final Mechanism2d mechanism2d;
  private final MechanismRoot2d elevatorRoot;
  private final MechanismLigament2d elevatorCarriage;

  // simulating a reverse limit switch
  // SparkLimitSwitchSim - did not work for me at all
  private final DIOSim bottomLimitSwitch;
  private final Debouncer limitSwitchDebouncer = new Debouncer(0.1);
  // private final SparkLimitSwitchSim bottomLimitSwitch;
  private boolean closedLoop = false;
  private double openLoopVoltage = 0.0;
  private double targetHeightInches = 0.0;
  private double currentHeightInches = 0.0;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("elevatorP", ElevatorConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("elevatorI", ElevatorConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("elevatorD", ElevatorConstants.kD);

  public ElevatorIOSim() {
    bottomLimitSwitch = new DIOSim(0);

    LinearSystem<N2, N1, N2> elevatorSystem =
        LinearSystemId.createElevatorSystem(
            motor,
            ElevatorConstants.carriageMass,
            ElevatorConstants.drumRadius,
            ElevatorConstants.gearRatio);

    elevatorSim =
        new ElevatorSim(
            elevatorSystem,
            motor,
            Units.inchesToMeters(ElevatorConstants.minElevatorHeight),
            Units.inchesToMeters(ElevatorConstants.maxElevatorHeight),
            false,
            0);

    leadSpark =
        new SparkMaxSim(new SparkMax(ElevatorConstants.leadCanId, MotorType.kBrushless), motor);
    followerSpark =
        new SparkMaxSim(new SparkMax(ElevatorConstants.followerCanId, MotorType.kBrushless), motor);

    leadEncoder = leadSpark.getRelativeEncoderSim();
    // TrapezoidProfile.Constraints constraints =
    //    new TrapezoidProfile.Constraints(
    //        ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
    controller = new PIDController(p.get(), i.get(), d.get());
    // feedforward =
    //    new ElevatorFeedforward(
    //        ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV,
    // ElevatorConstants.kA);

    // Setup Mechanism2d visualization
    mechanism2d = new Mechanism2d(40, ElevatorConstants.maxElevatorHeight);
    elevatorRoot = mechanism2d.getRoot("Elevator", 20, 0);
    elevatorCarriage = elevatorRoot.append(new MechanismLigament2d("Carriage", 0, 90));

    // bottomLimitSwitch =
    //    new SparkLimitSwitchSim(
    //       new SparkMax(ElevatorConstants.leadCanId, MotorType.kBrushless), false);

    SmartDashboard.putData("Elevator Simulation", mechanism2d);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    controller.setPID(p.get(), i.get(), d.get());
    inputs.leadSparkConnected = true;
    inputs.followerSparkConnected = true;

    elevatorSim.update(ElevatorConstants.kDt);

    currentHeightInches = Units.metersToInches(elevatorSim.getPositionMeters());

    leadEncoder.setPosition(currentHeightInches);

    // bottomLimitSwitch.setValue(this.currentHeightInches <= 0);
    boolean isAtBottom = limitSwitchDebouncer.calculate(currentHeightInches <= 0);
    bottomLimitSwitch.setValue(isAtBottom);

    // if (bottomLimitSwitch.getValue()) {
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
        // double ffOutput = feedforward.calculate(controller.getSetpoint().velocity);
        output = pidOutput; // + ffOutput;

        elevatorSim.setInput(output);
      }
    } else {
      output = this.openLoopVoltage;
      elevatorSim.setInput(output);
    }

    inputs.currentHeightPosition = this.currentHeightInches;
    inputs.targetHeightPosition = this.targetHeightInches;

    elevatorCarriage.setLength(inputs.currentHeightPosition);

    Logger.recordOutput("Elevator/BottomLimitSwitchPressed", bottomLimitSwitch.getValue());
    Logger.recordOutput("Elevator/LeadEncoderPosition", leadEncoder.getPosition());
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
