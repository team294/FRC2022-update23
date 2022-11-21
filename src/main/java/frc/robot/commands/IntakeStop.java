// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeStop extends CommandBase {
  /** Creates a new IntakeStop. */
  private Intake intake;
  private FileLog log;

  /**
   * Stops the intake motor(s).  If front intake, then stops both motors.
   * @param intake
   * @param log
   */
  public IntakeStop(Intake intake, FileLog log) {
    this.intake = intake;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, intake.getName(), "Stop");
    intake.stopMotor();
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
