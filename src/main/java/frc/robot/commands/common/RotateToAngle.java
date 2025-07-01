package frc.robot.commands.common;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class RotateToAngle extends Command {
  private final Drive drive;
  private final double targetAngle;
  private final PIDController pidController;

  private static final double kP = 5; // Proportional gain
  private static final double kI = 0.0; // Integral gain
  private static final double kD = 0.1; // Derivative gain
  private static final double TOLERANCE = 1.0; // Degrees
  private static final double MIN_ROTATION_SPEED = .2;
  private static final double MAX_ROTATION_SPEED = 3;

  public RotateToAngle(Drive drive, double targetAngle) {
    this.drive = drive;
    this.targetAngle = targetAngle;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.enableContinuousInput(-180, 180);
    this.pidController.setTolerance(TOLERANCE);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pidController.reset();
    pidController.setSetpoint(targetAngle);
  }

  @Override
  public void execute() {
    double currentAngle = drive.getPose().getRotation().getDegrees();
    double rotationSpeed = pidController.calculate(currentAngle);
    if (Math.abs(rotationSpeed) < MIN_ROTATION_SPEED) {
      rotationSpeed = 0;
    } else if (rotationSpeed > 0) {
      rotationSpeed = Math.max(MIN_ROTATION_SPEED, Math.min(rotationSpeed, MAX_ROTATION_SPEED));
    } else {
      rotationSpeed = Math.min(-MIN_ROTATION_SPEED, Math.max(rotationSpeed, -MAX_ROTATION_SPEED));
    }
    drive.runVelocity(new ChassisSpeeds(0, 0, rotationSpeed));
    System.out.println("Current Angle: " + currentAngle + ", Target Angle: " + targetAngle);
    System.out.println("Rotation Speed: " + rotationSpeed);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
