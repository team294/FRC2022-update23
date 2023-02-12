package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PhotonVision.CameraName;
import frc.robot.utilities.FileLog;

import java.util.List;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class DriveToTargetWithPhoton extends CommandBase {
  private final DriveTrain driveTrain;
  private final PhotonVision vision;
  private FileLog log = null;

  // PID constants should be tuned per robot
  private final double LINEAR_P = 0.1;
  private final double LINEAR_D = 0.0;
  private PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  private final double ANGULAR_P = 0.1;
  private final double ANGULAR_D = 0.0;
  private PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  private double range = -1;
  private double yaw = 360;

  /**
   * Creates a new DriveWithJoystick to control two motors.
   */
  public DriveToTargetWithPhoton(DriveTrain driveTrain, PhotonVision vision, FileLog log) {
    this.driveTrain = driveTrain;
    this.vision = vision;
    this.log = log;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    range = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forwardPct = 0;
    double rotationPct = 0;

    List<PhotonTrackedTarget> targets = vision.getTargets(CameraName.CENTER_CAMERA);

    if (targets != null && targets.size() > 0) {

      var target = targets.get(0);

      // prioritize key targets in the middle
      for (var t:targets) {
        if (t.getFiducialId() == 2 || t.getFiducialId() == 7) {
          target = t;
          break;
        }
      }

      log.writeLog(false, "Photon", "DriveToTargetWithVision", "Target found",target.getFiducialId());

      // First calculate range
      range = PhotonUtils.calculateDistanceToTargetMeters(
          PhotonVision.CAMERA_HEIGHT_METERS,
          PhotonVision.TARGET_HEIGHT_METERS,
          PhotonVision.CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(target.getPitch()));

      log.writeLog(false, "Photon", "DriveToTargetWithVision", "range", range);

      // Use this range as the measurement we give to the PID controller.
      // -1.0 required to ensure positive PID controller effort _increases_ range
      // forwardSpeed = -forwardController.calculate(range, 1);

      // Also calculate angular power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      yaw = target.getYaw();
      rotationPct = -turnController.calculate(yaw, 0);

    } else {
      // If we have no targets, stop and exit
      forwardPct = 0;
      rotationPct = 0;
      range = 0;
    }  
    
    log.writeLog(false, "Photon", "DriveToTargetWithVision", "forwardPct", forwardPct, "rotationPct", rotationPct);

    driveTrain.arcadeDrive(forwardPct, rotationPct);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "Photon", "DriveToTargetWithVision", "End", interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = false;
    if (Math.abs(yaw) < 3) {
      done = true;
      log.writeLog(false, "Photon", "DriveToTargetWithVision", "finished yaw", yaw);
    }
    return done;
  }

}
