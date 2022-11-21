// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.FileLog;

public class XboxRumble extends CommandBase {
  private Joystick xboxController;
  private double percentRumble;
  private double targetTime;
  private double startingTime;
  private FileLog log;
  private double seconds;
  private int numOfRumble;
  private int currRumble;
  /** Creates a new XboxRumble. */
  /**
   * Rumbles the Xbox Controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   * @param seconds time in seconds to rumble
   * @param numOfRumble how many rumbles.  If more than 1, then each rumble is "seconds" long, with a gap of "seconds" between rumbles.
   * @param xboxController xbox controller
   * @param log logger
   */
  public XboxRumble(double percentRumble, double seconds, int numOfRumble, Joystick xboxController, FileLog log) {
    this.xboxController = xboxController;
    this.log = log;
    this.percentRumble = MathUtil.clamp(Math.abs(percentRumble),0,1);
    this.seconds = Math.abs(seconds);
    this.numOfRumble = Math.abs(numOfRumble);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "XboxRumble", "Starting rumble", "seconds", seconds, "times", numOfRumble);
    currRumble = 1;
    startingTime = System.currentTimeMillis();
    targetTime = startingTime + (seconds * 1000 * (2*numOfRumble - 1) );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // log.writeLogEcho(false, "XboxRumble", "Execute", "Delta Time", (System.currentTimeMillis()-startingTime)/1000, "curRumble", currRumble);
    if( System.currentTimeMillis() < ( (2*currRumble-1) * seconds * 1000 + startingTime ) ) {
      // log.writeLogEcho(false, "XboxRumble", "Execute", "Rumbling", true);
      xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
      xboxController.setRumble(RumbleType.kRightRumble, percentRumble);
    } else if( System.currentTimeMillis() > ( 2 * currRumble * seconds * 1000 + startingTime ) ) {
      currRumble++;
    } else {
      // log.writeLogEcho(false, "XboxRumble", "Execute", "Rumbling", false);
      xboxController.setRumble(RumbleType.kLeftRumble, 0);
      xboxController.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "XboxRumble", "End", "Total time", (System.currentTimeMillis()-startingTime)/1000);
    xboxController.setRumble(RumbleType.kLeftRumble, 0);
    xboxController.setRumble(RumbleType.kRightRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (System.currentTimeMillis() > targetTime);
  }
}