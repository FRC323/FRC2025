// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import au.grapplerobotics.CanBridge;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auto.AutoCommands;
import frc.robot.commands.climb.ClimbCommands;
import frc.robot.commands.common.CommonCommands;
import frc.robot.commands.initialization.OffsetCommands;
import frc.robot.commands.initialization.ZeroGryo;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.commands.scoring.ScoreCommands;
import frc.robot.field.align.ReefAlignmentConstants.ReefPoleLabel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveConstants.DriveStick;
import frc.robot.subsystems.drive.DriveConstants.GamePad;
import frc.robot.subsystems.drive.DriveConstants.SteerStick;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intakes.algae.AlgaeIntake;
import frc.robot.subsystems.intakes.algae.AlgaeIntakeIOReal;
import frc.robot.subsystems.intakes.algae.AlgaeIntakeIOSim;
import frc.robot.subsystems.intakes.coral.CoralIntake;
import frc.robot.subsystems.intakes.coral.CoralIntakeIO;
import frc.robot.subsystems.intakes.coral.CoralIntakeIOReal;
import frc.robot.subsystems.intakes.coral.CoralIntakeIOSim;
import frc.robot.subsystems.intakes.ground.GroundIntake;
import frc.robot.subsystems.intakes.ground.GroundIntakeIO;
import frc.robot.subsystems.intakes.ground.GroundIntakeIOReal;
import frc.robot.subsystems.intakes.ground.GroundIntakeIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Vision vision;
  private final CoralIntake coralIntake;
  private final AlgaeIntake algaeIntake;
  private final GroundIntake groundIntake;
  private final Climber climber;

  private final CommandJoystick driveJoystick = new CommandJoystick(DriveConstants.DRIVE_STICK_PORT);
  private final CommandJoystick steerJoystick = new CommandJoystick(DriveConstants.STEER_STICK_PORT);
  private final CommandGenericHID gamePad = new CommandGenericHID(DriveConstants.GAME_PAD_PORT);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CanBridge.runTCP();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));

        elevator = new Elevator(new ElevatorIOSpark());
        arm = new Arm(new ArmIOSpark());
        coralIntake = new CoralIntake(new CoralIntakeIOReal());
        algaeIntake = new AlgaeIntake(new AlgaeIntakeIOReal());
        groundIntake = new GroundIntake(new GroundIntakeIOReal());
        climber = new Climber(new ClimberIOSpark());
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVision(
                VisionConstants.rearCameraName, VisionConstants.rearCameraToRobotTransform),
            new VisionIOPhotonVision(
                VisionConstants.frontCameraName, VisionConstants.frontCameraToRobotTransform),
            new VisionIOPhotonVision(
                VisionConstants.elevatorCameraName,
                VisionConstants.elevatorCameraToRobotTransform));

        if (VisionConstants.show2dField)
          SmartDashboard.putData("Field", drive.getField());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.rearCameraName,
                VisionConstants.rearCameraToRobotTransform,
                drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.frontCameraName,
                VisionConstants.frontCameraToRobotTransform,
                drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.elevatorCameraName,
                VisionConstants.elevatorCameraToRobotTransform,
                drive::getPose));

        coralIntake = new CoralIntake(new CoralIntakeIOSim());
        algaeIntake = new AlgaeIntake(new AlgaeIntakeIOSim());
        groundIntake = new GroundIntake(new GroundIntakeIOSim());
        climber = new Climber(new ClimberIOSim());

        SmartDashboard.putData("Field", drive.getField());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });

        elevator = new Elevator(new ElevatorIO() {
        });
        arm = new Arm(new ArmIO() {
        });
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        }, new VisionIO() {
        });
        coralIntake = new CoralIntake(new CoralIntakeIO() {
        });
        algaeIntake = new AlgaeIntake(new AlgaeIntakeIOReal() {
        });
        groundIntake = new GroundIntake(new GroundIntakeIO() {
        });
        climber = new Climber(new ClimberIOSpark() {
        });

        break;
    }

    // Register named commands
    registerNamedCommmands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Shuffleboard.getTab("SmartDashboard").add("Auton Chooser", autoChooser);

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive Wheel Radius Characterization",
    // DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    // "Drive Simple FF Characterization",
    // DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  // public Command autoDrive() {
  // return new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(0.8, 0, 0)),
  // drive)
  // .withTimeout(3.0);
  // }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            elevator,
            () -> -driveJoystick.getY(),
            () -> -driveJoystick.getX(),
            () -> -steerJoystick.getX()));

    coralIntake.setDefaultCommand(
        IntakeCommands.HoldCoralAndDetectScore(elevator, arm, coralIntake));
    algaeIntake.setDefaultCommand(IntakeCommands.HoldAlgaeIntake(arm, elevator, algaeIntake));

    SmartDashboard.putData(
        "AlignTag22Left", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.E));
    SmartDashboard.putData(
        "AlignTag22Right", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.F));
    SmartDashboard.putData(
        "AlignTag21Left", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.G));
    SmartDashboard.putData(
        "AlignTag21Right", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.H));
    SmartDashboard.putData("AlignHP12", AutoCommands.AlignToCoralStation12(drive, vision));
    SmartDashboard.putData("AlignBestTag", AutoCommands.AlignToReefBestTag(drive, vision));

    // reset gyro
    driveJoystick.button(DriveStick.RIGHT_SIDE_BUTTON).onTrue(new ZeroGryo(drive));

    steerJoystick
        .button(SteerStick.LEFT)
        .and(() -> Math.abs(driveJoystick.getX()) >= 0.2)
        .onTrue(AutoCommands.AlignToReef2(drive, vision, driveJoystick.getX()));

    // pose to reef algae intake level 1
    gamePad
        .povDown()
        .and(steerJoystick.button(SteerStick.MIDDLE))
        .whileTrue(
            IntakeCommands.AlgaeIntake(
                elevator,
                ElevatorPosition.REEF_LEVEL_1_ALGAE,
                arm,
                ArmPosition.REEF_LEVEL_1_ALGAE,
                algaeIntake));

    // pose to reef algae intake level 2
    gamePad
        .povUp()
        .and(steerJoystick.button(SteerStick.MIDDLE))
        .whileTrue(
            IntakeCommands.AlgaeIntake(
                elevator,
                ElevatorPosition.REEF_LEVEL_2_ALGAE,
                arm,
                ArmPosition.REEF_LEVEL_2_ALGAE,
                algaeIntake));

    // shoot into barge
    steerJoystick
        .button(SteerStick.RIGHT)
        .whileTrue(
            ScoreCommands.ScoreAlgae(
                elevator, ElevatorPosition.ALGAE_BARGE, arm, ArmPosition.ALGAE_BARGE));

    // pose to algae processor
    steerJoystick
        .trigger()
        .whileTrue(
            IntakeCommands.AlgaeIntake(
                elevator,
                ElevatorPosition.ALGAE_PROCESSOR,
                arm,
                ArmPosition.ALGAE_PROCESSOR,
                algaeIntake));

    // pose to reef coral level 1
    gamePad
        .button(GamePad.A_BUTTON)
        .and(steerJoystick.button(SteerStick.MIDDLE))
        .whileTrue(
            ScoreCommands.ScoreCoral(
                elevator,
                ElevatorPosition.REEF_LEVEL_1_CORAL,
                arm,
                ArmPosition.REEF_LEVEL_1_CORAL,
                groundIntake));

    // pose to reef coral level 2
    gamePad
        .button(GamePad.B_BUTTON)
        .and(steerJoystick.button(SteerStick.MIDDLE))
        .whileTrue(
            ScoreCommands.ScoreCoral(
                elevator,
                ElevatorPosition.REEF_LEVEL_2_CORAL,
                arm,
                ArmPosition.REEF_LEVEL_2_CORAL,
                groundIntake));

    // pose to reef coral level 3
    gamePad
        .button(GamePad.X_BUTTON)
        .and(steerJoystick.button(SteerStick.MIDDLE))
        .whileTrue(
            ScoreCommands.ScoreCoral(
                elevator,
                ElevatorPosition.REEF_LEVEL_3_CORAL,
                arm,
                ArmPosition.REEF_LEVEL_3_CORAL,
                groundIntake));

    // pose to reef coral level 4
    gamePad
        .button(GamePad.Y_BUTTON)
        .and(steerJoystick.button(SteerStick.MIDDLE))
        .whileTrue(
            ScoreCommands.ScoreCoral(
                elevator,
                ElevatorPosition.REEF_LEVEL_4_CORAL,
                arm,
                ArmPosition.REEF_LEVEL_4_CORAL,
                groundIntake));

    // pose to human player coral pickup
    // driveJoystick.trigger().onTrue(IntakeCommands.CoralIntake(elevator, arm,
    // coralIntake));

    groundIntake.setDefaultCommand(IntakeCommands.StopGroundIntake(groundIntake));

    // pose to ground pick up and turn on intakes
    driveJoystick
        .trigger()
        .whileTrue(IntakeCommands.MoveToGroundPickup(elevator, arm, groundIntake, coralIntake));

    // spit out algae
    driveJoystick
        .button(DriveStick.TOP_BIG_BUTTON)
        .whileTrue(IntakeCommands.OuttakeAlgae(algaeIntake));

    // spit out coral - no assistance
    driveJoystick
        .button(DriveStick.LEFT_SIDE_BUTTON)
        .whileTrue(IntakeCommands.OuttakeCoral(coralIntake, groundIntake));

    driveJoystick
        .button(DriveStick.SMALL_TOP_BUTTON)
        .whileTrue(
            IntakeCommands.MoveToHumanPlayerPickup(elevator, arm, groundIntake, coralIntake));

    // spit out coral - no assistance
    // driveJoystick
    // .button(DriveStick.LEFT_SIDE_BUTTON)
    // .whileTrue(IntakeCommands.OuttakeCoral(coralIntake));

    // temp reset
    driveJoystick
        .button(DriveStick.BACK_SIDE_BUTTON)
        .onTrue(
            CommonCommands.moveToTravelPosition(
                elevator, arm, coralIntake, algaeIntake, groundIntake));

    climber.setDefaultCommand(ClimbCommands.ClimberStop(climber));

    // deploy climber
    driveJoystick
        .button(DriveStick.LEFT_DIRECTIONAL)
        .whileTrue(ClimbCommands.MoveClimberToDeploy(climber, elevator, arm, groundIntake));

    driveJoystick
        .button(DriveStick.UP_DIRECTIONAL)
        .whileTrue(ClimbCommands.MoveClimberToDonkeyKong(climber));

    driveJoystick
        .button(DriveStick.DOWN_DIRECTIONAL)
        .whileTrue(ClimbCommands.MoveClimberToStow(climber));

    // ROBOT INITIALIZE COMMANDS
    SmartDashboard.putData("Set Drivetrain Offsets", OffsetCommands.storeDrivetrainOffsets(drive));
  }

  private void registerNamedCommmands() {
    NamedCommands.registerCommand(
        "Coral L1 Score",
        ScoreCommands.ScoreCoral(
            elevator,
            ElevatorPosition.REEF_LEVEL_2_CORAL,
            arm,
            ArmPosition.REEF_LEVEL_2_CORAL,
            groundIntake)
            .alongWith(IntakeCommands.RunCoralIntake2(coralIntake).withTimeout(1.5)));
    NamedCommands.registerCommand(
        "Coral L2 Score",
        ScoreCommands.ScoreCoral(
            elevator,
            ElevatorPosition.REEF_LEVEL_2_CORAL,
            arm,
            ArmPosition.REEF_LEVEL_2_CORAL,
            groundIntake)
            .alongWith(IntakeCommands.RunCoralIntake2(coralIntake).withTimeout(1.5)));
    NamedCommands.registerCommand(
        "Coral L3 Score",
        ScoreCommands.ScoreCoral(
            elevator,
            ElevatorPosition.REEF_LEVEL_3_CORAL,
            arm,
            ArmPosition.REEF_LEVEL_3_CORAL,
            groundIntake)
            .alongWith(IntakeCommands.RunCoralIntake2(coralIntake).withTimeout(1.5)));
    NamedCommands.registerCommand(
        "Coral L4 Score",
        ScoreCommands.ScoreCoral(
            elevator,
            ElevatorPosition.REEF_LEVEL_4_CORAL,
            arm,
            ArmPosition.REEF_LEVEL_4_CORAL,
            groundIntake)
            .alongWith(IntakeCommands.RunCoralIntake2(coralIntake).withTimeout(1.5)));
    NamedCommands.registerCommand(
        "Coral HP Intake",
        AutoCommands.MoveToCoralIntakeAuto(elevator, arm, coralIntake, groundIntake));
    NamedCommands.registerCommand(
        "Travel",
        CommonCommands.moveToTravelPosition(
            elevator, arm, coralIntake, algaeIntake, groundIntake));

    NamedCommands.registerCommand("Run Coral Outtake", AutoCommands.CoralOuttakeAuto(coralIntake));
    NamedCommands.registerCommand("Run Coral Intake", AutoCommands.CoralIntakeAuto(coralIntake));

    NamedCommands.registerCommand(
        "ReefAlign_A", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.A));
    NamedCommands.registerCommand(
        "ReefAlign_B", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.B));
    NamedCommands.registerCommand(
        "ReefAlign_C", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.C));
    NamedCommands.registerCommand(
        "ReefAlign_D", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.D));
    NamedCommands.registerCommand(
        "ReefAlign_E", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.E));
    NamedCommands.registerCommand(
        "ReefAlign_F", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.F));
    NamedCommands.registerCommand(
        "ReefAlign_G", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.G));
    NamedCommands.registerCommand(
        "ReefAlign_H", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.H));
    NamedCommands.registerCommand(
        "ReefAlign_I", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.I));
    NamedCommands.registerCommand(
        "ReefAlign_J", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.J));
    NamedCommands.registerCommand(
        "ReefAlign_K", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.K));
    NamedCommands.registerCommand(
        "ReefAlign_L", AutoCommands.AlignToReef(drive, vision, ReefPoleLabel.L));

    NamedCommands.registerCommand(
        "CoralStationAlign_12", AutoCommands.AlignToCoralStation12(drive, vision));
    NamedCommands.registerCommand(
        "CoralStationAlign_13", AutoCommands.AlignToCoralStation13(drive, vision));
    NamedCommands.registerCommand(
        "CoralStationAlign_1", AutoCommands.AlignToCoralStation1(drive, vision));
    NamedCommands.registerCommand(
        "CoralStationAlign_2", AutoCommands.AlignToCoralStation2(drive, vision));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void stopAllSubsystems() {
    elevator.stop();
    arm.stop();
    algaeIntake.stop();
    coralIntake.stop();
    groundIntake.stop();
    climber.stop();
  }
}
