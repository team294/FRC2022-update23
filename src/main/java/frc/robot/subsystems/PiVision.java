// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.StringUtil;

public class PiVision extends SubsystemBase implements Loggable {
  private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  protected NetworkTable table;
  protected NetworkTableEntry rx, ry, rv, rw;
  protected FileLog log;
  protected double x, y, width, numberOfTargets;
  protected String name;
  protected int networkTableReadCounter = 0;
  protected boolean fastLogging = false;

  /** Creates a new PiVision. */
  public PiVision(String name, FileLog log) {
    this.log = log;
    this.name = name;
    table = tableInstance.getTable("photonvision").getSubTable("Microsoft_LifeCam_HD-3000");

    rw = table.getEntry("targetPixelsX");
    rx = table.getEntry("targetPixelsX");
    // ry = table.getEntry("ry");
    rv = table.getEntry("targetPixelsX");
  }

  /**
   * @return true when pivision sees a target, false when not seeing a target
   */
  public boolean seesTarget() {
    return (numberOfTargets>0);
  }

  /**
   * @return true when pivision is connected & reading data
   * false when pivisionhub is disconnected or not reading any data   TODO when would x be 1064??
   */
  public boolean isGettingData() {
    // return (x != (1000 * PiVisionConstants.angleMultiplier) && y != 1000);
    return (x != (1000) && width != 1000 && numberOfTargets != 1000);
  }

  /**
   * @return offset of target in degrees from center, positive = clockwise
   */
  public double getXOffset() {
    return -x;
  }

  /**
   * reads data from network tables
   */
  public void readData() {
    double xNew, yNew, numberOfTargetsNew, widthNew; 
    numberOfTargetsNew = rv.getDouble(1000.0);
    widthNew = rw.getDouble(1000.0);
    // xNew = rx.getDouble(1000.0) * PiVisionConstants.angleMultiplier;
    xNew = rx.getDouble(1000.0);
    // yNew = ry.getDouble(1000.0);
    networkTableReadCounter = 0;
  
    // Check if the pivision updated the NetworkTable while we were reading values, to ensure that all
    // of the data (targetExists, X, Y, etc) are from the same vision frame.
    do {
      numberOfTargets = numberOfTargetsNew;
      width = widthNew;
      x = xNew;
      // y = yNew;

      numberOfTargetsNew = rv.getDouble(1000.0);
      widthNew = rw.getDouble(1000.0);
      xNew = rx.getDouble(1000.0);
      // yNew = ry.getDouble(1000.0);
      networkTableReadCounter++;
    // } while(networkTableReadCounter<= 5 && (xNew != x || yNew != y));
    } while(networkTableReadCounter<= 5 && (xNew != x || widthNew != width || numberOfTargetsNew != numberOfTargets));

    // Convert X from pixels to an angle
    x = -(x - 320.0) / 12.0;
  }

  @Override
  public void periodic() {
    readData();
    
    if (fastLogging || log.getLogRotation() == log.PIVISION_CYCLE) {
      updatePiVisionLog(false);
    }
    
    if (log.getLogRotation() == log.PIVISION_CYCLE) {

      if(!isGettingData()) {
        // TODO robot preferences
        // RobotPreferences.recordStickyFaults(name, log);
        log.writeLog(false, name, "Update Variables", "Failure", "NOT WORKING");
      }

      // SmartDashboard.putNumber(StringUtil.buildString(name, " area"), area);
      SmartDashboard.putNumber(StringUtil.buildString(name, " x"), x);
      // SmartDashboard.putNumber(StringUtil.buildString(name, " y"), y);
      SmartDashboard.putNumber(StringUtil.buildString(name, " width"), width);
      SmartDashboard.putNumber(StringUtil.buildString(name, " targets"), numberOfTargets);
      SmartDashboard.putBoolean(StringUtil.buildString(name, " Updating"), isGettingData());
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about pivision to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updatePiVisionLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, name, "Update Variables", 
      "Number of Targets", numberOfTargets, // number of targets
      "Target Width", width, 
      "Center Offset X", x, 
      // "Center Offset Y", y,
      // "Target Area", area,
      // "Latency", latency,
      "Network Table Read Counter", networkTableReadCounter
      // "Snapshot Count", snapshotCount
      );
  }
}
