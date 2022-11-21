// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;


public class ShooterSetPercentOutput extends CommandBase {

  private final Shooter shooter;
  private final FileLog log;
  private double percentOut = 0.0;
  private boolean fromShuffleboard = false;

  /**
   * Sets the motor speed from Shuffboard, -1.0 to +1.0 (+ = shoot out)
   * @param shooter motor subsystem
   * @param log logfile
   */
  public ShooterSetPercentOutput(Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    fromShuffleboard = true;

    SmartDashboard.putNumber(buildString(shooter.getName(), " Percent"), 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  /**
   * Sets the motor speed
   * @param percentOut -1.0 to +1.0, + = shoot out
   * @param shooter motor subsystem
   * @param log logfile
   */
  public ShooterSetPercentOutput(double percentOut, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.percentOut = percentOut;
    fromShuffleboard = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      percentOut = SmartDashboard.getNumber(buildString(shooter.getName(), " Percent"), 0);
    } else {
      SmartDashboard.putNumber(buildString(shooter.getName(), " Percent"), percentOut);
    }

    log.writeLog(false, "ShooterSetPercentOutput", "Initialize", "percent", percentOut);
    shooter.setMotorPercentOutput(percentOut);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
