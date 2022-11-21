// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;


public class TurretRampOutput extends CommandBase {

  private final Turret turret;
  private final FileLog log;
  private double startPercent = 0.0, stopPercent = 0.5;
  private double stepPercent = 10.0;
  private double percentOut = 0.0;

  /**
   * Ramps the motor percent from startPercent to stopPercent over rampTimeSec seconds.
   * After the ramp, the motor speed is held at stopPercent until the command
   * is cancelled.
   * @param startPercent motor percent, -1.0 to 1.0
   * @param stopPercent motor percent, -1.0 to 1.0
   * @param rampTimeSec ramp time, in seconds
   * @param turret turret subsystem
   * @param log logfile
   */
  public TurretRampOutput(double startPercent, double stopPercent, double rampTimeSec,
      Turret turret, FileLog log) {
    this.turret = turret;
    this.log = log;
    this.startPercent = startPercent;
    this.stopPercent = stopPercent;
    stepPercent = (stopPercent - startPercent)*0.020/rampTimeSec;   // Execute is called every 20msec

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percentOut = startPercent;
    turret.enableFastLogging(true);

    log.writeLog(false, turret.getName(), "RampOutput Start", "Start Percent", startPercent, 
        "Stop Percent", stopPercent, "stepPercent", stepPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber(buildString(turret.getName(), " Percent"), percentOut);
    turret.setPercentOutput(percentOut);

    if ( ( (stepPercent>0) && (percentOut<stopPercent) ) ||
         ( (stepPercent<0) && (percentOut>stopPercent) ) ) {
      percentOut += stepPercent;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();

    log.writeLog(false, turret.getName(), "RampOutput Finish", "Final Percent", percentOut);
    turret.enableFastLogging(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
