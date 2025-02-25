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
  // private final TrapezoidProfile.Constraints constraints =
  //     new TrapezoidProfile.Constraints(
  //         ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
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

  private double targetPosition = 0.0;
  private double openLoopVoltage = 0.0;
  private boolean closedLoop = false;

  private final LoggedNetworkNumber p = new LoggedNetworkNumber("elevatorP", ElevatorConstants.kP);
  private final LoggedNetworkNumber i = new LoggedNetworkNumber("elevatorI", ElevatorConstants.kI);
  private final LoggedNetworkNumber d = new LoggedNetworkNumber("elevatorD", ElevatorConstants.kD);

  // private final ElevatorFeedforward feedforward =
  //     new ElevatorFeedforward(
  //         ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV,
  // ElevatorConstants.kA);

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
            true,
            0);

    leadSpark =
        new SparkMaxSim(new SparkMax(ElevatorConstants.leadCanId, MotorType.kBrushless), motor);
    followerSpark =
        new SparkMaxSim(new SparkMax(ElevatorConstants.followerCanId, MotorType.kBrushless), motor);

    leadEncoder = leadSpark.getRelativeEncoderSim();
    controller = new PIDController(p.get(), i.get(), d.get());

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

    var currentPosition = Units.metersToInches(elevatorSim.getPositionMeters());

    leadEncoder.setPosition(currentPosition);

    inputs.leadEncoderPosition = currentPosition;

    boolean isAtBottom = limitSwitchDebouncer.calculate(currentPosition <= 0);
    bottomLimitSwitch.setValue(isAtBottom);

    if (bottomLimitSwitch.getValue()) {
      elevatorSim.setInput(0);
      elevatorSim.setState(0, 0);
      inputs.homed = true;
      inputs.isAtBottom = true;
    } else {
      inputs.isAtBottom = false;
    }

    double output = 0;
    if (closedLoop) {
      if (inputs.homed) {
        double pidOutput = controller.calculate(currentPosition, this.targetPosition);
        // double ffOutput = feedforward.calculate(controller.getSetpoint().velocity);
        output = pidOutput; // + ffOutput;

        elevatorSim.setInput(output);
      }
    } else {
      output = this.openLoopVoltage;
      elevatorSim.setInput(output);
    }

    elevatorCarriage.setLength(currentPosition);

    Logger.recordOutput("Elevator/ControlOutput", output);
    Logger.recordOutput("Elevator/BottomLimitSwitchPressed", bottomLimitSwitch.getValue());
    Logger.recordOutput("Elevator/LeadEncoderPosition", leadEncoder.getPosition());
    Logger.recordOutput("Elevator/CurrentPosition", leadEncoder.getPosition());
  }

  @Override
  public void setPercent(double percent) {
    this.closedLoop = false;
    var clamped = MathUtil.clamp(percent, -1.0, 1.0);
    setVoltage(clamped * 12);
  }

  @Override
  public void setVoltage(double voltage) {
    this.closedLoop = false;
    var clamped = MathUtil.clamp(voltage, -12, 12);
    openLoopVoltage = clamped;
  }

  @Override
  public void setPosition(double position) {
    this.closedLoop = true;
    this.targetPosition = position;
  }
}
