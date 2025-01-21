package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorIOSim implements ElevatorIO {
    private final DCMotor elevatorMotor = DCMotor.getNEO(2);

    private final ProfiledPIDController controller =
        new ProfiledPIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(
                ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration));

        ElevatorFeedforward m_feedforward =
            new ElevatorFeedforward(
                ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private final Encoder encoder =
        new Encoder(ElevatorConstants.elevatorPortLead);

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorMotor,
          ElevatorConstants.gearRatio,
          ElevatorConstants.carriageMass,
          ElevatorConstants.drumRadius,
          ElevatorConstants.minElevatorHeightMeters,
          ElevatorConstants.maxElevatorHeightMeters,
          true,
          0,
          0.01,
          0.0);

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d mech2d = new Mechanism2d(22.5, 60);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
  private final MechanismLigament2d elevatorMech2d =
      mech2dRoot.append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  private double targetHeightInches = 0.0;

    public ElevatorIOSim() {
        SmartDashboard.putData("Elevator Sim", mech2d);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leadSparkConnected = true;
        inputs.followerSparkConnected = true;

        double controlEffort = controller.calculate(inputs.ElevatorHeightInches, targetHeightInches);
        elevatorSim.setInput(controlEffort);

        elevatorSim.update(ElevatorConstants.kDt);

        Logger.recordOutput("Eelevator/controlEffort", controlEffort);
        Logger.recordOutput("Elevator/actual controller kp", controller.getP());

        inputs.ElevatorHeightInches = Units.metersToInches(elevatorSim.getPositionMeters());
        inputs.ElevatorTargetHeightInches = targetHeightInches;

        elevatorMech2d.setLength(encoder.getDistance());
    }

    @Override
    public void setHeightInches(double targetHeightInches) {
        this.targetHeightInches = targetHeightInches;
    }
}
