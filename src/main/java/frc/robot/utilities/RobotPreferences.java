/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

/**
 * This class handles all stored robot preferences
 */
public class RobotPreferences {
    private static String problemSubsystem = "";
    private static boolean problemExists = false;

    /**
     * Reads preferences stored in the RoboRIO flash memory and updates their values in Constants.
     * If a preference is not found on the RoboRIO, then this method creates that key on the RoboRio
     * using the default values from Constants.
     * Note:  Any variables in Constants that are read from Preferences must *not* be "final".
     */
	public static void readPreferencesToConstants(){
        // Add a row for each preference to read.
        // RobotConstants.prototypeBot = readBoolean("prototypeBot", RobotConstants.prototypeBot);

        // LimeLightConstants.takeSnapshots = readBoolean("Limelight.takeSnapshots", LimeLightConstants.takeSnapshots);

        // DriveConstants.ticksPerInch = readDouble("Drive.ticksPerInch", DriveConstants.ticksPerInch);
        // DriveConstants.TRACK_WIDTH = readDouble("Drive.TrackWidth", DriveConstants.TRACK_WIDTH);

        // DriveConstants.kMaxSpeedMetersPerSecond = readDouble("Drive.kMaxSpeedMPS", DriveConstants.kMaxSpeedMetersPerSecond);
        // DriveConstants.kMaxAccelerationMetersPerSecondSquared = readDouble("Drive.kMaxAccelerationMPSS", DriveConstants.kMaxAccelerationMetersPerSecondSquared);
        // DriveConstants.kSLinear = readDouble("Drive.kSLinear", DriveConstants.kSLinear);
        // DriveConstants.kVLinear = readDouble("Drive.kVLinear", DriveConstants.kVLinear);
        // DriveConstants.kALinear = readDouble("Drive.kALinear", DriveConstants.kALinear);
        // DriveConstants.kPLinear = readDouble("Drive.kPLinear", DriveConstants.kPLinear);
        // DriveConstants.kILinear = readDouble("Drive.kILinear", DriveConstants.kILinear);
        // DriveConstants.kDLinear = readDouble("Drive.kDLinear", DriveConstants.kDLinear);
        // DriveConstants.kAngLinear = readDouble("Drive.kAngLinear", DriveConstants.kAngLinear);

        // DriveConstants.kMaxAngularVelocity = readDouble("Drive.kMaxAngularVelocityDegS", DriveConstants.kMaxAngularVelocity);
        // DriveConstants.kMaxAngularAcceleration = readDouble("Drive.kMaxAngularAccelerationDegSS", DriveConstants.kMaxAngularAcceleration);
        // DriveConstants.kSAngular = readDouble("Drive.kSAngular", DriveConstants.kSAngular);
        // DriveConstants.kVAngular = readDouble("Drive.kVAngular", DriveConstants.kVAngular);
        // DriveConstants.kAAngular = readDouble("Drive.kAAngular", DriveConstants.kAAngular);
        // DriveConstants.kPAngular = readDouble("Drive.kPAngular", DriveConstants.kPAngular);
        // DriveConstants.kIAngular = readDouble("Drive.kIAngular", DriveConstants.kIAngular);
        // DriveConstants.kDAngular = readDouble("Drive.kDAngular", DriveConstants.kDAngular);
        // DriveConstants.tLagAngular = readDouble("Drive.tLagAngular", DriveConstants.tLagAngular);
        
        // DriveConstants.updateDerivedConstants();
    }

    /**
     * Reads a boolean key from the RoboRIO preferences.  If the key does not exist on the 
     * RoboRIO, then this method creates the key on the RoboRIO with the default value
     * (and in that case also returns the default value).
     * @param keyName Name of the RoboRIO preferences key
     * @param defaultValue Default value if the key doesn't already exist
     * @return Value from RoboRIO preferences, or default value if it didn't exist
     */
    private static boolean readBoolean(String keyName, boolean defaultValue) {
		if (!Preferences.containsKey(keyName)){
			Preferences.setBoolean(keyName, defaultValue);
        } 
        return Preferences.getBoolean(keyName, defaultValue);   
    }

    /**
     * Reads a double key from the RoboRIO preferences.  If the key does not exist on the 
     * RoboRIO, then this method creates the key on the RoboRIO with the default value
     * (and in that case also returns the default value).
     * @param keyName Name of the RoboRIO preferences key
     * @param defaultValue Default value if the key doesn't already exist
     * @return Value from RoboRIO preferences, or default value if it didn't exist
     */
    private static double readDouble(String keyName, double defaultValue) {
		if (!Preferences.containsKey(keyName)){
            Preferences.setDouble(keyName, defaultValue);
        } 
        return Preferences.getDouble(keyName, defaultValue);   
    }

    /**
     * Reads a string key from the RoboRIO preferences.  If the key does not exist on the 
     * RoboRIO, then this method creates the key on the RoboRIO with the default value
     * (and in that case also returns the default value).
     * @param keyName Name of the RoboRIO preferences key
     * @param defaultValue Default value if the key doesn't already exist
     * @return Value from RoboRIO preferences, or default value if it didn't exist
     */
    private static String readString(String keyName, String defaultValue) {
		if (!Preferences.containsKey(keyName)){
            Preferences.setString(keyName, defaultValue);
        } 
        return Preferences.getString(keyName, defaultValue);   
    }

    /**
	 * Records in robotPreferences, fileLog, and Shuffleboard that a problem was found in a subsystem
	 * (only records if the subsystem wasn't already flagged)
	 * @param subsystem String name of subsystem in which a problem exists
	 */
	public static void recordStickyFaults(String subsystem, FileLog log) {
		if (problemSubsystem.indexOf(subsystem) == -1) {
			if (problemSubsystem.length() == 0) {
			    problemSubsystem = subsystem;
			} else {
				problemSubsystem = StringUtil.buildString(problemSubsystem, ", ", subsystem);
            }
			log.writeLogEcho(true, subsystem, "Sticky Faults Logged", "");
		}
		if (!problemExists) {
			problemExists = true;
		}
		editStickyFaultsInPrefs(problemSubsystem, problemExists);
		showStickyFaults();
	}
	
	/**
	 * Clears any sticky faults in the RobotPreferences and Shuffleboard
	 */
	public static void clearStickyFaults(FileLog log) {
		problemSubsystem = "";
		problemExists = false;
        editStickyFaultsInPrefs(problemSubsystem, problemExists);
        showStickyFaults();
		log.writeLog(true, "StickyFaults", "Sticky Faults Cleared", "");
	}

	/**
	 * Show any sticky faults on Shuffleboard
	 */
	public static void showStickyFaults() {
		SmartDashboard.putString("problemSubsystem", problemSubsystem);
		SmartDashboard.putBoolean("problemExists", problemExists);
	}

    private static void editStickyFaultsInPrefs(String problemSubsystem, boolean problemExists) {
        Preferences.setString("problemSubsystem", problemSubsystem);
        Preferences.setBoolean("problemExists", problemExists);
    }

    /**
     * Verifies that RobotPreferences is not empty (reset all preferences by accident)
     * @return true = preferences exist on robot
     */
    public static boolean prefsExist() {
        return Preferences.containsKey("problemSubsystem");
    }
}
