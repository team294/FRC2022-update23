/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.*;

/**
 * Command to control the drive train with joysticks using arcade drive.
 */
public class DriveWithJoystickArcade extends CommandBase {
  private final DriveTrain driveTrain;
  private final Joystick leftJoystick;
  private final Joystick rightJoystick;
  private final FileLog log;
  
  private double fwdPercent, turnPercent;
  private double lastFwdPercent, lastTime, curTime;

  private final double maxFwdRateChange = 2.0;
  private final double maxRevRateChange = -1.4;

  /**
   * @param driveTrain drive train subsystem to use
   * @param leftJoystick left joystick
   * @param rightJoystick right joystick
   * @param log filelog to use
   */
  public DriveWithJoystickArcade(DriveTrain driveTrain, Joystick leftJoystick, Joystick rightJoystick, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);
    driveTrain.setOpenLoopRampLimit(false);

    lastFwdPercent = 0;
    lastTime = System.currentTimeMillis() / 1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curTime = System.currentTimeMillis() / 1000.0;
    fwdPercent = -leftJoystick.getY();
    turnPercent = -rightJoystick.getX() * 0.5;

    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      log.writeLog(false, "DriveWithJoystickArcade", "Joystick", "L Joystick", fwdPercent, "R Joystick", turnPercent);
    }

    double fwdRateChange = (fwdPercent - lastFwdPercent) / (curTime - lastTime);
    if (fwdRateChange > maxFwdRateChange) {
      fwdPercent = lastFwdPercent + (curTime - lastTime)*maxFwdRateChange;
    } else if (fwdRateChange < maxRevRateChange) {
      fwdPercent = lastFwdPercent +(curTime - lastTime)*maxRevRateChange;

    }
    
    driveTrain.arcadeDrive(fwdPercent, turnPercent);

    lastFwdPercent = fwdPercent;
    lastTime = curTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
