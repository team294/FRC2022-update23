// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeToFeeder extends CommandBase {
  private Uptake uptake;
  private Feeder feeder;
  private FileLog log;
  /** Creates a new UptakeToFeeder. */
  public UptakeToFeeder(Uptake uptake, Feeder feeder, FileLog log) {
    this.uptake = uptake;
    this.feeder = feeder;
    this.log = log;
    addRequirements(uptake);
    addRequirements(feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "UptakeToFeeder", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    uptake.setEjectPercentOutput(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(feeder.isBallPresent()){
      uptake.setEjectPercentOutput(0);
      return true;
    }
    else return false;
  }
}
