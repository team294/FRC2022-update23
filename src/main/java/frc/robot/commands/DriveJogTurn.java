/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.*;
import edu.wpi.first.wpilibj.Timer;

public class DriveJogTurn extends CommandBase {
  private final DriveTrain driveTrain;
  private final FileLog log;
  private final Timer timer;
  
  private double jogPercent = 0.15;  
  private double jogTime = 0.05;//  was 0.1
  private boolean jogRight = false;

  /**
   * @param driveTrain drive train subsystem to use
   * @param jogRight   jogs right if true,  left if false
   * @param log filelog to use
   */
  public DriveJogTurn(boolean jogRight, DriveTrain driveTrain, FileLog log )  {
    this.driveTrain = driveTrain;
    this.log = log;
    this.jogRight = jogRight;
    this.timer = new Timer();
   
    addRequirements(driveTrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "DriveJogTurn", "Start");
    timer.reset();
    timer.start();
    driveTrain.setOpenLoopRampLimit(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (jogRight) driveTrain.arcadeDrive(0, jogPercent);
    else driveTrain.arcadeDrive(0, -jogPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "DriveJogTurn", "End");
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
    driveTrain.setDriveModeCoast(false);
    driveTrain.setOpenLoopRampLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return timer.hasElapsed(jogTime); // terminate when time has elapsed
  }
}
