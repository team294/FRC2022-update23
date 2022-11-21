// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;


public class TurretSetPercentOutput extends CommandBase {

  private final Turret turret;
  private final FileLog log;
  private double percentOut = 0.0;
  private boolean fromShuffleboard = false;

  /**
   * Sets the turret speed from Shuffboard, -1.0 to +1.0 (+ = turn right).
   * This command immediately ends, but it leaves the turret running
   * at the requested speed.  To stop the turret, run the TurretStop command.
   * @param turret turret subsystem
   * @param log logfile
   */
  public TurretSetPercentOutput(Turret turret, FileLog log) {
    this.turret = turret;
    this.log = log;
    fromShuffleboard = true;

    SmartDashboard.putNumber(buildString(turret.getName(), " Percent"), 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  /**
   * Sets the turret speed
   * @param percentOut -1.0 to +1.0, + = turn right
   * @param turret turret subsystem
   * @param log logfile
   */
  public TurretSetPercentOutput(double percentOut, Turret turret, FileLog log) {
    this.turret = turret;
    this.log = log;
    this.percentOut = percentOut;
    fromShuffleboard = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      percentOut = SmartDashboard.getNumber(buildString(turret.getName(), " Percent"), 0);
    } else {
      SmartDashboard.putNumber(buildString(turret.getName(), " Percent"), percentOut);
    }

    log.writeLog(false, turret.getName(), "SetPercentOutput", "Percent", percentOut);
    turret.setPercentOutput(percentOut);
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
