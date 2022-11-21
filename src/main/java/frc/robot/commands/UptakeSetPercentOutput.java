// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class UptakeSetPercentOutput extends CommandBase {
  /** Creates a new testUptake. */
  private final Uptake uptake;
  private FileLog log;
  private double uptakePercent, ejectPercent;

  /**
   * Runs the uptake and eject motors
   * @param percent -1.0 to 1.0, + = up, - = down
   * @param ejectBall true = ball path to eject, false = ball path to feeder
   * @param uptake uptake subsystem
   * @param log logfile
   */
  public UptakeSetPercentOutput(double percent, boolean ejectBall, Uptake uptake, FileLog log) {
    this.uptake = uptake;
    this.log = log;

    uptakePercent = percent;
    ejectPercent = ejectBall ? percent : -percent;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
  }

  /**
   * Runs the uptake and eject motors
   * @param uptakePercent -1.0 to 1.0, + = up, - = down
   * @param ejectPercent -1.0 to 1.0, + = eject ball, - = send ball to feeder
   * @param uptake uptake subsystem
   * @param log logfile
   */
  public UptakeSetPercentOutput(double uptakePercent, double ejectPercent, Uptake uptake, FileLog log) {
    this.uptakePercent = uptakePercent;
    this.ejectPercent = ejectPercent;
    this.uptake = uptake;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
  }

  /**
   * Runs the uptake and eject motors
   * @param uptakePercent -1.0 to 1.0, + = up, - = down
   * @param ejectPercent -1.0 to 1.0, + = eject ball, - = send ball to feeder
   * @param uptake uptake subsystem
   * @param log logfile
   * @param requireUptakeSubsystem true = requires uptake subsystem (normal usage).  false = does not 
   * require uptake subsystem (use with EXTREME care)
   */
  public UptakeSetPercentOutput(double uptakePercent, double ejectPercent, Uptake uptake, FileLog log, boolean requireUptakeSubsystem) {
    this.uptakePercent = uptakePercent;
    this.ejectPercent = ejectPercent;
    this.uptake = uptake;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    if (requireUptakeSubsystem) {
      addRequirements(uptake);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    uptake.setUptakePercentOutput(uptakePercent);
    uptake.setEjectPercentOutput(ejectPercent);

    log.writeLog(false, "Uptake", "initialize", "Uptake Percent", uptakePercent, "Eject Percent", ejectPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
