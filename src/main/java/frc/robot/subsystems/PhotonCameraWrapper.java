/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.utilities.Field;
import frc.robot.utilities.FileLog;

import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Field field;
  private FileLog log;

  public PhotonCameraWrapper(Field field, FileLog log) {
    this.log = log;
    this.field = field;
  }

  public void init() {
    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Starting");

    if (photonCamera == null) {
      photonCamera = new PhotonCamera(VisionConstants.cameraName);
    }

    aprilTagFieldLayout = field.getAprilTagFieldLayout();
    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Loaded april tags from field");
    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", 
      "AT8 x",aprilTagFieldLayout.getTagPose(8).get().getX(),
      "AT8 y",aprilTagFieldLayout.getTagPose(8).get().getY(),
      "AT8 rot",aprilTagFieldLayout.getTagPose(8).get().getRotation());

    // try {
    //   aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    //   log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Loaded april tags from file");
    // } catch (IOException e) {
    //   log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Error loading april tags from file");
    //   e.printStackTrace();
    // }

    // Create pose estimator
    photonPoseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
        photonCamera,
        VisionConstants.robotToCam);

    log.writeLogEcho(true, "PhotonCameraWrapper", "Init", "Done");
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    log.writeLogEcho(true, "PhotonCameraWrapper", "getEstimatedGlobalPose", "PreviousPose", "X",prevEstimatedRobotPose.getX(),"Y",prevEstimatedRobotPose.getY());
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    var newPoseOptional = photonPoseEstimator.update();
    if (newPoseOptional.isPresent()) {
      EstimatedRobotPose newPose = newPoseOptional.get();
      log.writeLogEcho(true, "PhotonCameraWrapper", "getEstimatedGlobalPose", "NewPose", "X",newPose.estimatedPose.getX(),"Y",newPose.estimatedPose.getY());
    }
    return newPoseOptional;
  }
}