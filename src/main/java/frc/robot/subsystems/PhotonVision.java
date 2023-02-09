package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

public class PhotonVision extends SubsystemBase implements Loggable {

  // Constants such as camera and target height stored. Change per robot and goal!
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

  // Angle between horizontal and the camera.
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  private PhotonCamera camera = new PhotonCamera(VisionConstants.rightCameraName);
  private PhotonPoseEstimator photonPoseEstimator;
  private FileLog log = null;

  // Set up a test arena of two apriltags at the center of each driver station set
  private final AprilTag tag03 = new AprilTag(3, new Pose3d( new Pose2d(0,Units.feetToMeters(5.42), Rotation2d.fromDegrees(180))));
  private final AprilTag tag01 = new AprilTag(01,new Pose3d(new Pose2d(0.0, 0, Rotation2d.fromDegrees(0.0))));
  private final ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

  public PhotonVision(FileLog log) {
    this.log = log;
    atList.add(tag03);
    atList.add(tag01);

    AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

    // Create pose estimator
    photonPoseEstimator=new PhotonPoseEstimator(atfl,PoseStrategy.CLOSEST_TO_REFERENCE_POSE,camera,VisionConstants.robotToCam);
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
    if (pose.isPresent()) {
      log.writeLog(false, "Photon",
        "Pose X", pose.get().estimatedPose.getX(),
        "Pose Y", pose.get().estimatedPose.getY(),
        "Pose Z", pose.get().estimatedPose.getZ());
    } else {
      log.writeLog(false, "Photon", "EstimatedPose", "null");
    }
    
    return pose;
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    // ignore for now
  }

  @Override
  public void periodic() {

    try {
      var latest = camera.getLatestResult();
      

      if (latest != null) {
        var target = latest.getBestTarget();
        if (target != null) {
          log.writeLogEcho(false, "Photon",
          "Area", target.getArea(),
          "Pitch", target.getPitch(),
          "Pose Z", target.getSkew(),
          "Yaw", target.getYaw());
          SmartDashboard.putNumber("Photon Area", target.getArea());
          SmartDashboard.putNumber("Photon Pitch", target.getPitch());
          SmartDashboard.putNumber("Photon Skew", target.getSkew());
          SmartDashboard.putNumber("Photon Yaw", target.getYaw());
        } else {
          log.writeLog(false, "Photon", "No target");
        }
      } else {
        log.writeLog(false, "Photon", "No result");
      }
   } catch (Exception e) {
    log.writeLogEcho(false, "Photon","error",e.getMessage());
   }
    
    // if (latest.isPresent()) {
    //   log.writeLog(false, "Photon",
    //   "Periodic Pose X", newpose.get().estimatedPose.getX(),
    //   "Pose Y", newpose.get().estimatedPose.getY(),
    //   "Pose Z", newpose.get().estimatedPose.getZ());
    // } else {
    //   log.writeLog(false, "Photon", "No newpose");  
    // }
    

  }

  public void initialize() {
    resetPose();
  }

  public void resetPose(Pose2d robotPoseInMeters) {
    odometry.resetPosition(Rotation2d.fromDegrees(0), 0, 0, robotPoseInMeters);
  }

  public void resetPose() {
    odometry.resetPosition(Rotation2d.fromDegrees(0), 0, 0, new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
  }
  
}
