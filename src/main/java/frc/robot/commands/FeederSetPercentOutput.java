// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;


public class FeederSetPercentOutput extends CommandBase {

  private final Feeder feeder;
  
  private double percentOut = 0.0;
  private boolean fromShuffleboard = false;
  private FileLog log;
  

  /**
   * Sets the motor speed
   * @param percentOut -1.0 to +1.0, + = towards shooter
   * @param feeder feeder subsystem
   * @param log logfile
   */
  public FeederSetPercentOutput(double percentOut, Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;
    this.percentOut = percentOut;
    fromShuffleboard = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  /**
   * Sets the motor speed from Shuffboard
   * @param feeder Feeder subsystem
   * @param log logfile
   */
  public FeederSetPercentOutput(Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;
    SmartDashboard.putNumber(buildString(feeder.getName(), " Percent"), 0);
    fromShuffleboard = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromShuffleboard) {
      percentOut = SmartDashboard.getNumber(buildString(feeder.getName(), " Percent"), 0);
    } else {
      SmartDashboard.putNumber(buildString(feeder.getName(), " Percent"), percentOut);
    }

    log.writeLog(false, feeder.getName(), "SetPercentOutput", "Percent", percentOut);
    feeder.setMotorPercentOutput(percentOut);
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
