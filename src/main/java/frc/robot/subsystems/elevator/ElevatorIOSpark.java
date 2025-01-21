package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.sparkStickyFault;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax leadSpark;
  private final SparkMax followerSpark;

  private final Debouncer leadConnectedDebounce = new Debouncer(0.5);
  private final Debouncer followerConnectedDebounce = new Debouncer(0.5);

  private final AbsoluteEncoder encoder;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          constraints,
          ElevatorConstants.kDt);

  public double getHeightInInches() {
    return 0.0;
  }

  public ElevatorIOSpark() {
    leadSpark = new SparkMax(ElevatorConstants.elevatorPortLead, MotorType.kBrushless);
    followerSpark = new SparkMax(ElevatorConstants.elevatorPortFollower, MotorType.kBrushless);

    encoder = leadSpark.getAbsoluteEncoder();
    // follower set inverted
    // set up configs
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sparkStickyFault = false;
    inputs.leadSparkConnected = leadConnectedDebounce.calculate(!sparkStickyFault);
    inputs.followerSparkConnected = followerConnectedDebounce.calculate(!sparkStickyFault);
  }
}
