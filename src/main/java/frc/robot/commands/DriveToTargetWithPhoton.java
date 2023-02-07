package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.utilities.FileLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class DriveToTargetWithPhoton extends CommandBase {
  private final DriveTrain driveTrain;
  private final PhotonVision vision;

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  /**
   * Creates a new DriveWithJoystick to control two motors.
   */
  public DriveToTargetWithPhoton(DriveTrain driveTrain, PhotonVision vision, FileLog log) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    addRequirements(driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forwardSpeed;
    double rotationSpeed;
    double range;
    double targetRange = 1;

    var result = vision.getLatestResult();

    if (result.hasTargets()) {
      // First calculate range
      range = PhotonUtils.calculateDistanceToTargetMeters(
          PhotonVision.CAMERA_HEIGHT_METERS,
          PhotonVision.TARGET_HEIGHT_METERS,
          PhotonVision.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch()));

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(range, targetRange);

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      // If we have no targets, stay still.
      forwardSpeed = 0;
      rotationSpeed = 0;
      range = 0;
    }

    // post to smart dashboard periodically
    SmartDashboard.putNumber("Photon-targetRange", targetRange);
    SmartDashboard.putNumber("Photon-range", range);
    SmartDashboard.putNumber("Photon-forwardSpeed", forwardSpeed);
    SmartDashboard.putNumber("Photon-rotationSpeed", rotationSpeed);

    driveTrain.arcadeDrive(forwardSpeed, rotationSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
