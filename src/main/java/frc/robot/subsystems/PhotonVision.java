package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Loggable;

public class PhotonVision extends SubsystemBase implements Loggable {

  // Constants such as camera and target height stored. Change per robot and goal!
  public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

  // Angle between horizontal and the camera.
  public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  private PhotonCamera camera = new PhotonCamera("photonvision");

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    // TODO Auto-generated method stub
    
  }
  
}
