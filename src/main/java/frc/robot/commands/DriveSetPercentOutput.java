/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveSetPercentOutput extends CommandBase {
  /**
   * Sets drive motors to percent output
   */

   private DriveTrain driveTrain;
   private FileLog log;
   private double lPercent;
   private double rPercent;
  public DriveSetPercentOutput(double lPercent, double rPercent, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.lPercent = lPercent;
    this.rPercent = rPercent;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setOpenLoopRampLimit(true);
    driveTrain.setLeftMotorOutput(lPercent);
    driveTrain.setRightMotorOutput(rPercent);
    log.writeLog(false, "DriveSetPercentOutput", "Init", "Target % L", lPercent, "Target % R", rPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.setLeftMotorOutput(lPercent);
    driveTrain.setRightMotorOutput(rPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}