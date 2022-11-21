/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.StringUtil;

import static frc.robot.Constants.LimeLightConstants.*;

public class LimeLight extends SubsystemBase implements Loggable {
  private NetworkTableInstance tableInstance = NetworkTableInstance.getDefault();
  protected NetworkTable table;
  protected NetworkTableEntry tv, tx, ty, ta, tl, pipeline, ledMode, camMode;
  protected String limelightName;
  protected double targetExists, x, y, area;
  protected double latency;
  protected double pipe;
  protected FileLog log;
  protected Timer snapshotTimer;
  protected int snapshotCount = 0;
  protected int networkTableReadCounter = 0;
  protected boolean fastLogging = false;

  /*
   * Limelight settings: ~~input~~ Exposure: 2 Black level offset: 0 red balance:
   * 1200 blue balance: 1975 ~~thresholding~~ hue: 60 - 95 saturation: 150 - 255
   * value: 170 - 255 erosion steps: 0 dilation steps: 0 ~~countour filtering~~
   * area % image: 0.0163 - 100 fullness % of blue rectangle: 10 - 100 w/h ratio:
   * 0 - 20 direction filter : none smart specle detection: 0 target grouping:
   * single target intersection filter: none ~~output~~ targeting region: center
   * send raw corners? : nah send raw contours: no crosshair mode: Single
   * crosshair x: 0 y: 0 ~~3d experimental~~ no changes
   */
  public LimeLight(String tableName, FileLog log) {
    this.log = log;
    table = tableInstance.getTable(tableName);
    limelightName = StringUtil.buildString(tableName.substring(0, 0).toUpperCase(), tableName.substring(1) );

    this.snapshotTimer = new Timer();
    snapshotTimer.reset();
    snapshotTimer.start();
    tableInstance.startClientTeam(294);

    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tl = table.getEntry("tl");
    pipeline = table.getEntry("pipeline");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
  }

  /**
   * @return horizontal (x-axis) angle, in degrees, between camera crosshair and target crosshair
   * + = target is to the left, - = target is to the right
   */
  public double getXOffset() {
    return x;
  }

  /**
   * @return vertical (y-axis) angle, in degrees, between camera crosshair and target crosshair
   * down is negative, up is positive
   */
  public double getYOffset() {
    return y;
  }

  /**
   * @return area of target
   */
  public double getArea() {
    return area;
  }

  /**
   * @return latency contribution by pipeline
   */
  public double getLatency() {
    return latency;
  }


  /**
   * @return current pipeline number
   */
  public double getPipeline() {
    return table.getEntry("getpipe").getDouble(0);
  }

  /**
   * @param pipeNum 0 is vision, 2 is driver cam
   */
  public void setPipe(double pipeNum) {
    pipeline.setDouble(pipeNum);
  }

  /**
   * Sets the green LEDs on the front of the limelight.
   * <ul><li>0 = the LED Mode set in the current pipeline (find this at the ip address of the limelight).</li>
   * <li>1 = off</li>
   * <li>2 = blink</li>
   * <li>3 = on</li></ul>
   * If a number other than 0 to 3 is selected, turn the LEDs off.
   * @param modeNumber select a number from 0 to 3.
   */
  public void setLedMode(int modeNumber) {
    if (modeNumber > 3 || modeNumber < 0) modeNumber = 1;
    ledMode.setDouble(modeNumber);
  }

  /**
   * Sets the mode of the camera for use as driver cam or vision processing
   * @param mode 0 = vision; 1 = driver camera (increases exposure, disables vision processing)
   */
  public void setCamMode(int mode) {
      camMode.setDouble(mode);
  }


  /**
   * @param snapshot true is take a snapshot, false is don't take snapshots
   * will increment "snapshotCount" to keep track of how many snapshots have been taken
   * should only be called if canTakeSnapshot() is true
   */
  public void setSnapshot(boolean snapshot) {
    // TODO disable snapshots -- this seems to cause camera lag
    snapshot = false;

    if(snapshot && canTakeSnapshot()) {
      table.getEntry("snapshot").setNumber(1);
      if(table.getEntry("snapshot").getDouble(0)==1) {
        snapshotCount++;
      }
      log.writeLog(false, limelightName, "setSnapshot", "Snapshot Count", snapshotCount);

      snapshotTimer.reset();
      snapshotTimer.start();
    }
    else table.getEntry("snapshot").setNumber(0);
  }

  /**
   * Limelight will only save snapshots every 0.5 seconds
   * @return true if enough time has passed since the last snapshot to take another snapshot
   *              in order to prevent overloading the Limelight with snapshot requests
   *              "takeSnapshots" has to be true in preferences (meaning we want to be taking snapshots)
   *         false if we shouldn't take a snapshot (either due to delay or not wanting to take snapshots)
   */
  public boolean canTakeSnapshot() {
    return snapshotTimer.hasElapsed(0.5) && takeSnapshots;
  }

  /**
   * reset number of snapshots taken (as if we are clearing our cache of snapshots)
   * This will NOT actually delete the snapshots (we don't have that capability in code)
   */
  public void resetSnapshotCount() {
    snapshotCount = 0;
  }


 
//TODO increase limelight logging and edit seesTarget to check last three periodic for target
  /**
   * @return true when limelight sees a target, false when not seeing a target
   */
  public boolean seesTarget() {
    return (targetExists==1);
  }

  /**
   * @return true when limelight is connected & reading data
   * false when limelight is disconnected or not reading any data   TODO when would x be 1064??
   */
  public boolean isGettingData() {
    return (x != (1000 * angleMultiplier) && y != 1000);
  }

  public void readData () {
    double targetExistsNew, xNew, yNew, areaNew, latencyNew; 
    targetExistsNew = tv.getDouble(1000.0);
    xNew = -tx.getDouble(1000.0) * angleMultiplier;
    yNew = ty.getDouble(1000.0);
    areaNew = ta.getDouble(1000.0);
    latencyNew = tl.getDouble(1000.0);
    networkTableReadCounter = 0;
  
    // Check if the Limelight updated the NetworkTable while we were reading values, to ensure that all
    // of the data (targetExists, X, Y, etc) are from the same vision frame.
    do {
      targetExists = targetExistsNew;
      x = xNew;
      y = yNew;
      area = areaNew;
      latency = latencyNew;

      targetExistsNew = tv.getDouble(1000.0);
      xNew = -tx.getDouble(1000.0) * angleMultiplier;
      yNew = ty.getDouble(1000.0);
      areaNew = ta.getDouble(1000.0);
      latencyNew = tl.getDouble(1000.0);
      networkTableReadCounter++;
    } while(networkTableReadCounter<= 5 && (targetExistsNew != targetExists || xNew != x || yNew != y
    || areaNew != area || latencyNew != latency));

  }

  @Override
  public void periodic() {
    readData();
    
    if (fastLogging || log.getLogRotation() == log.LIMELIGHT_CYCLE) {
      updateLimeLightLog(false);
    }
    
    if (log.getLogRotation() == log.LIMELIGHT_CYCLE) {

      // turn off for 2022 as we are not using this for anything critical
      // if(!isGettingData()) {
      //   RobotPreferences.recordStickyFaults(limelightName, log);
      // }

      // Invert X on SmartDashboard, since bars on SmartDashboard always go from - (left) to + (right)
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " area"), area);
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " x"), -x);
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " y"), y);
      SmartDashboard.putBoolean(StringUtil.buildString(limelightName, " Sees Target"), seesTarget());
      SmartDashboard.putBoolean(StringUtil.buildString(limelightName, " Updating"), isGettingData());
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " Latency"), getLatency());
      SmartDashboard.putNumber(StringUtil.buildString(limelightName, " Snapshot Count"), snapshotCount);
      
      pipe = SmartDashboard.getNumber(StringUtil.buildString(limelightName, " Pipeline"), 0); // default is vision pipeline

      if (getPipeline() != pipe) {
        log.writeLogEcho(false, limelightName, "Pipeline change", "Pipeline", pipe);
        setPipe(pipe);
      }
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about limelight to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLimeLightLog(boolean logWhenDisabled) {
    log.writeLog(logWhenDisabled, limelightName, "Update Variables", 
      "Target Valid", seesTarget(),
      "Center Offset X", x, 
      "Center Offset Y", y,
      "Target Area", area,
      "Latency", latency,
      "Network Table Read Counter", networkTableReadCounter,
      "Snapshot Count", snapshotCount
      );
  }
}
