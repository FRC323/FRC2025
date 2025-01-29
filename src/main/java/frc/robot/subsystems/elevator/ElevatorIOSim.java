package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor elevatorMotor = DCMotor.getNEO(2);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

  @SuppressWarnings("rawtypes")
  private final LinearSystem elesys =
      LinearSystemId.createElevatorSystem(
          elevatorMotor,
          ElevatorConstants.carriageMass,
          ElevatorConstants.drumRadius,
          ElevatorConstants.gearRatio);

  @SuppressWarnings("unchecked")
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elesys,
          elevatorMotor,
          Units.inchesToMeters(ElevatorConstants.minElevatorHeightInches),
          Units.inchesToMeters(ElevatorConstants.maxElevatorHeightInches),
          true,
          0);

  private final Mechanism2d mech2d;
  private final MechanismRoot2d simRobotBase;
  private final MechanismLigament2d elevator;

  private double targetHeightInches = 0.0;

  public ElevatorIOSim() {
    mech2d = new Mechanism2d(22.5, 60);
    simRobotBase = mech2d.getRoot("SimRobotBase", 0, 0);

    MechanismLigament2d base =
        simRobotBase.append(new MechanismLigament2d("Base", 22.5, 0, 40, new Color8Bit(0, 0, 255)));
    elevator =
        base.append(new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit(255, 0, 0)));

    SmartDashboard.putData("Elevator Mechanism", mech2d);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // System.out.println(
    // "velocity: "
    // + this.controller.getSetpoint().velocity
    // + " | position: "
    // + this.controller.getSetpoint().position);

    double currentHeightInches = Units.metersToInches(elevatorSim.getPositionMeters());

    double controlEffort = controller.calculate(currentHeightInches, targetHeightInches);
    // + feedforward.calculate(this.controller.getSetpoint().velocity);

    // System.out.println(
    // "currentHeightInches: "
    // + currentHeightInches
    // + " | targetHeightInches: "
    // + targetHeightInches);

    elevatorSim.setInput(controlEffort);
    elevatorSim.update(ElevatorConstants.kDt);
    elevator.setLength(currentHeightInches);

    inputs.leadSparkConnected = true;
    inputs.followerSparkConnected = true;
    inputs.ElevatorHeightInches = currentHeightInches;
    inputs.ElevatorTargetHeightInches = targetHeightInches;

    Logger.recordOutput("Elevator/ControlEffort", controlEffort);
    Logger.recordOutput("Elevator/CurrentHeightInches", currentHeightInches);
    Logger.recordOutput("Elevator/TargetHeightInches", this.targetHeightInches);
  }

  @Override
  public void setHeightInches(double targetHeightInches) {
    this.targetHeightInches = targetHeightInches;
    this.controller.setGoal(targetHeightInches);
  }
}
