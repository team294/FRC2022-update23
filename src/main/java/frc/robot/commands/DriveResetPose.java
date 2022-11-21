/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveResetPose extends CommandBase {
  /**
   * Resets the pose, gyro, and encoders on the drive train
   */

  private DriveTrain driveTrain;
  private FileLog log;
  private double curX, curY, curAngle;

  /**
	 * Resets the pose, gyro, and encoders on the drive train
   * @param curXinMeters Robot X location in the field, in meters (0 = middle of robot wherever the robot starts auto mode, +=away from our drivestation)
   * @param curYinMeters Robot Y location in the field, in meters (0 = middle of robot wherever the robot starts auto mode, +=left when looking from our drivestation)
   * @param curAngleinDegrees Robot angle on the field, in degrees (0 = facing away from our drivestation)
   * @param driveTrain DriveTrain subsytem
	 */
  public DriveResetPose(double curXinMeters, double curYinMeters, double curAngleinDegrees, DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    curX = curXinMeters;
    curY = curYinMeters;
    curAngle = curAngleinDegrees;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "DriveResetPose", "Init", "Curr X", curX, "CurrY", curY, "CurAng", curAngle);
    driveTrain.resetPose(new Pose2d(curX, curY, new Rotation2d(Math.toRadians(curAngle))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
