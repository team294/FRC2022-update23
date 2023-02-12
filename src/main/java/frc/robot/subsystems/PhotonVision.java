package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
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

public class PhotonVision extends SubsystemBase {

  // Update these depending upon the robot
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(33);
  public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(24.38); // 24.38in to bottom of april tag
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  public static enum CameraName {CENTER_CAMERA, LEFT_CAMERA};
  private PhotonCamera[] cameraArray = new PhotonCamera[CameraName.values().length];

  private PhotonPipelineResult[] resultArray = new PhotonPipelineResult[CameraName.values().length];

  // private PhotonPoseEstimator photonPoseEstimator;
  private FileLog log = null;
  private long lastUpdate = 0;

  // Set up a test arena of two apriltags at the center of each driver station set
  // private final AprilTag tag03 = new AprilTag(3, new Pose3d( new Pose2d(0,Units.feetToMeters(5.42), Rotation2d.fromDegrees(180))));
  // private final AprilTag tag01 = new AprilTag(01,new Pose3d(new Pose2d(0.0, 0, Rotation2d.fromDegrees(0.0))));
  // private final ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
  // private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

  

  public PhotonVision(FileLog log) {
    this.log = log;

    cameraArray[CameraName.CENTER_CAMERA.ordinal()] = new PhotonCamera("CenterCamera");
    cameraArray[CameraName.LEFT_CAMERA.ordinal()] = new PhotonCamera("LeftCamera");

    // atList.add(tag03);
    // atList.add(tag01);
    //AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);
    // Create pose estimator
    //photonPoseEstimator=new PhotonPoseEstimator(atfl,PoseStrategy.CLOSEST_TO_REFERENCE_POSE,camera,VisionConstants.robotToCam);
  }

  public void initialize() {
    // resetPose();
  }

  @Override
  public void periodic() {

    try {
      var current = System.currentTimeMillis();
      if ((current - lastUpdate) >= 100) {
        lastUpdate = current;
        updateVision(PhotonVision.CameraName.LEFT_CAMERA);
        updateVision(PhotonVision.CameraName.CENTER_CAMERA);
      }
    } catch (Exception e) {
      log.writeLogEcho(false, "Photon","periodic", "error",e.getMessage());
    }

  }

  private void updateVision(CameraName cameraName) {
    var cameraIndex = cameraName.ordinal();
    // log.writeLogEcho(false, "Photon", "updateVision","cameraName",cameraName, "cameraIndex", cameraIndex);
    resultArray[cameraIndex] = cameraArray[cameraIndex].getLatestResult();

    if (resultArray[cameraIndex] != null && resultArray[cameraIndex].hasTargets()) {
      if (resultArray[cameraIndex].targets == null || resultArray[cameraIndex].targets.size() < 1) {
        log.writeLog(false, "Photon", "updateVision", "Targets", 0);
        SmartDashboard.putNumber("Photon Targets", 0);
      } else {
        log.writeLog(false, "Photon", "updateVision", "Targets", resultArray[cameraIndex].targets.size());
        SmartDashboard.putNumber("Photon Targets", resultArray[cameraIndex].targets.size());

        for (var target:resultArray[cameraIndex].targets) {

          // First calculate range
          double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(target.getPitch()));  

          log.writeLog(false, "Photon", "updateVision",
            "tagId",target.getFiducialId(),
            "Yaw", target.getYaw(),
            "Range", range,
            "Pitch", target.getPitch(),
            "Area", target.getArea(),
            "Skew", target.getSkew(),
            "Ambiguity",target.getPoseAmbiguity());
          
          // double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
            
          // Calculate robot's field relative pose
          // Pose2D robotPose = PhotonUtils.estimateFieldToRobot(
          //   kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);

          // Calculate robot's field relative pose
          // Pose3D robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);

        }
      }
    } else {
      //log.writeLog(false, "Photon", "updateVision", "No result");
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

  public List<PhotonTrackedTarget> getTargets(CameraName cameraName) {
    return resultArray[cameraName.ordinal()].targets;
  }



  // public void resetPose(Pose2d robotPoseInMeters) {
  //   odometry.resetPosition(Rotation2d.fromDegrees(0), 0, 0, robotPoseInMeters);
  // }

  // public void resetPose() {
  //   odometry.resetPosition(Rotation2d.fromDegrees(0), 0, 0, new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
  // }


  // /**
  //  * @param estimatedRobotPose The current best guess at robot pose
  //  * @return A pair of the fused camera observations to a single Pose2d on the
  //  *         field, and the time
  //  *         of the observation. Assumes a planar field and the robot is always
  //  *         firmly on the ground
  //  */
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
  //   photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
  //   Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
  //   if (pose.isPresent()) {
  //     log.writeLog(false, "Photon","EstimatedPose",
  //       "Pose X", pose.get().estimatedPose.getX(),
  //       "Pose Y", pose.get().estimatedPose.getY(),
  //       "Pose Z", pose.get().estimatedPose.getZ());
  //   } else {
  //     log.writeLog(false, "Photon", "EstimatedPose", "null");
  //   }
    
  //   return pose;
  // }  
  
}
