
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.PiVisionHub;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterSetVelocity extends CommandBase {

  public enum InputMode {
    kSpeedRPM,
    kDistFeet,    // TODO 9 feet for (3400) medium shot, short shot (3100) 7 feet
    kLastSetSpeed; 
  }
  
  private Shooter shooter;
  private PiVisionHub piVisionHub;
  private FileLog log;
  private boolean fromShuffleboard;
  private double velocity;
  private InputMode mode;

  private int toleranceCounter = 0;
  /**
   * Sets the shooter to a specific velocity using the PID controller.
   * The velocity is read from Shuffleboard, depending on the mode:
   * InputMode.kSpeedRPM = "Shooter SetPoint RPM"
   * InputMode.kDistFeet = "Shooter Distance Feet"
   * InputMode.kLastSetSpeed = don't use Shuffleboard but use the last speed the shooter was set to
   * @param mode InputMode.kSpeedRPM or InputMode.kDistFeet
   * @param shooter shooter subsystem
   * @param log log file
   */
  public ShooterSetVelocity(InputMode mode, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.mode = mode;
    this.fromShuffleboard = true;
    piVisionHub = null;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    if (mode == InputMode.kDistFeet) {
      SmartDashboard.putNumber("Shooter Distance Feet", 5.0);
    } else if(mode == InputMode.kSpeedRPM) {
      SmartDashboard.putNumber("Shooter SetPoint RPM", 2000.0);
    }
  }

  /**
   * Sets the shooter to a specific velocity using the PID controller.
   * @param mode InputMode.kSpeedRPM or InputMode.kDistFeet or InputMode.kLastSetSpeed -- Defines units for "value" parameter
   * @param value setpoint speed (in RMP) or distance (in feet)
   * @param shooter shooter subsystem
   * @param log log file
   */
  public ShooterSetVelocity(InputMode mode, double value, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.mode = mode;
    this.fromShuffleboard = false;
    piVisionHub = null;

    log.writeLog(false, "Shooter", "SetVelocity", "mode", mode, "value", value,"fromShuffleboard", fromShuffleboard);
    if (mode == InputMode.kDistFeet) {
      velocity = shooter.distanceFromTargetToRPM(value);
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity from distance", velocity);
    } else if(mode == InputMode.kSpeedRPM) {
      velocity = value;
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity from rpm", velocity);
    } else {
      velocity = shooter.getSetpointRPM();
      log.writeLog(false, "Shooter", "SetVelocity", "Velocity last set value", velocity);
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
  /**
   * Sets the shooter to a specific velocity using the PID controller.
   * Velocity is determined from PiVisionHub
   * @param shooter shooter subsystem
   * @param PiVisionHub camera
   * @param log log file
   */
  public ShooterSetVelocity(Shooter shooter, PiVisionHub pivisionhub, FileLog log) {
    this.shooter = shooter;
    this.piVisionHub = pivisionhub;
    this.log = log;
    this.mode = InputMode.kDistFeet;
    this.fromShuffleboard = false;
    
    log.writeLog(false, "Shooter", "SetVelocity", "from pi", true);
    velocity = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      if (mode==InputMode.kDistFeet) {
        double dist = SmartDashboard.getNumber("Shooter Distance Feet", 5.0);
        velocity = shooter.distanceFromTargetToRPM(dist);
        log.writeLog(false, "Shooter SetVelocity", "Initialize", "VelocityFromShuffleboardDistance", velocity);
        SmartDashboard.putNumber("Shoter SetPoint RPM", velocity);
      } else if (mode == InputMode.kSpeedRPM) {
        velocity = SmartDashboard.getNumber("Shooter SetPoint RPM", 0.0);
        log.writeLog(false, "Shooter SetVelocity", "Initialize", "VelocityFromShuffleboardRPM", velocity);
      }
    } else {
      if (piVisionHub != null) {
        velocity = shooter.distanceFromTargetToRPM(piVisionHub.getDistance());
        log.writeLog(false, "Shooter SetVelocity", "Initialize", "VelocityFromVision", velocity);
      } 
    }
    
    if(velocity >= 2000 || mode==InputMode.kLastSetSpeed){
      shooter.enableFastLogging(true);
    }

    if (mode==InputMode.kLastSetSpeed) {
      // Don't change the speed setting.  Just wait for the shooter to get to the previously set speed and then exit. 
      log.writeLog(false, "Shooter SetVelocity", "Initialize", "Using Last Velocity", velocity);
    } else {
      shooter.setMotorVelocity(velocity);
      log.writeLog(false, "Shooter SetVelocity", "Initialize", "Set Velocity", velocity);
      SmartDashboard.putNumber("Shooter SetPoint RPM", velocity);
    }

    toleranceCounter =0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);
    log.writeLog(false, "Shooter SetVelocity", "End", "Velocity", shooter.getMotorVelocity());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(shooter.getVelocityPIDError()) < ShooterConstants.pidErrorTolerance) {
      log.writeLogEcho(false, "Shooter SetVelocity", "Set RPM within tolerance", "Set Velocity", velocity, "Velocity", shooter.getMotorVelocity());
      toleranceCounter ++;

    } else {
      toleranceCounter = 0;
    }
      return (toleranceCounter >= 5);
  }
}
