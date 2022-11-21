/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.FileLog;

public class PiVisionHub extends PiVision {
  private PowerDistribution pd;
  
  public PiVisionHub(PowerDistribution pd, FileLog log) {
    super("shooter-cam", log);
    this.pd = pd;
  }

  /**
   * Sets LED state
   * @param state true for on, false for off
   */
  public void setLEDState(boolean state) {
    pd.setSwitchableChannel(state);
  }

  /**
   * @return gets current LED state, false = off
   */
  public boolean LEDState() {
    return pd.getSwitchableChannel();
  }

  /**
   * Takes into account not being in line with the target.
   * @return distance from camera to target, on the floor, in feet
   */
  public double getDistance() {
    if (width == 0) return 0;
    double w = width * ((36.5)/(numberOfTargets*5+(numberOfTargets-1)*5.5)); 
    return 26691.7 / w - 60.0122;  
  }

  @Override
  public void periodic() {
    super.periodic();
    
    if (log.getLogRotation() == log.PIVISION_CYCLE) {
      SmartDashboard.putNumber("cam-shooter Distance", getDistance());
    }
  }
  /**
   * Write information about limelight to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  @Override
  public void updatePiVisionLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, name, "Update Variables", 
      "Number of Targets", numberOfTargets,
      "Width", width,
      "X Offset", x,
      // "Target Area", area,
      // "Latency", latency,
      "Network Table Read Counter", networkTableReadCounter,
      // "Snapshot Count", snapshotCount,
      "Dist", getDistance()
      );
  }
}
