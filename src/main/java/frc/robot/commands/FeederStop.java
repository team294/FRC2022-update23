// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class FeederStop extends CommandBase {
  
  private Feeder feeder;
  private final FileLog log;

  /**
   * Stops the feeder motor
   * @param feeder feeder subsystem
   * @param log logfile
   */
  public FeederStop(Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, feeder.getName(), "Stop");
    feeder.stopMotor();
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
