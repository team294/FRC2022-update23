/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterDistToRPM extends CommandBase {
  private Shooter shooter;
  private FileLog log;
  private double distance, rpm;
  private boolean fromShuffleboard;

  /**
   * Calculate shooter RPM using distance away from target, with Shuffleboard input.
   * This command will not end if sample distances are from Shuffleboard (for continuous calculations).
   * @param shooter shooter subsystem
   */
  public ShooterDistToRPM(Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.distance = 0;
    this.fromShuffleboard = true;
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter Distance Feet", -9999) == -9999)
      SmartDashboard.putNumber("Shooter Distance Feet", 5);
  }

  /**
   * Calculate shooter RPM using distance away from target, with parameter distance.
   * @param distance distance from target, in feet
   * @param shooter shooter subsystem
   */
  public ShooterDistToRPM(double distance, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.distance = distance;
    this.fromShuffleboard = false;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) distance = SmartDashboard.getNumber("Shooter Distance Feet", 5);
    rpm = shooter.distanceFromTargetToRPM(distance);
    SmartDashboard.putNumber("Shooter RPM from Dist", rpm);
    log.writeLog(false, "ShooterDistToRPM", "Init", "TargetRPM", rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = SmartDashboard.getNumber("Shooter Distance Feet", 5);
    rpm = shooter.distanceFromTargetToRPM(distance);
    SmartDashboard.putNumber("Shooter RPM from Dist", rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !fromShuffleboard;
  }
}
