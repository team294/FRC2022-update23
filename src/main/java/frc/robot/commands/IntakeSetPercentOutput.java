// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeFront;
import frc.robot.utilities.FileLog;

public class IntakeSetPercentOutput extends CommandBase {
  /** Creates a new IntakeSetPercentOutput. */
  private Intake intake;
  private double intakePercent, transferPercent;
  private FileLog log;

  /**
   * Sets the percent speed of the intake motor and the transfer motor.  If rear intake, then the transfer
   * motor is ignored.
   * @param intakePercent  -1 to +1  (+ = intake, - = outtake)
   * @param transferPercent  -1 to +1  (+ = intake, - = outtake)
   * @param intake  Intake subsystem
   * @param log
   */
  public IntakeSetPercentOutput(double intakePercent, double transferPercent, Intake intake, FileLog log) {
    this.intake = intake;
    this.intakePercent = intakePercent;
    this.transferPercent = transferPercent;
    this.log = log;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  /**
   * Sets the percent speed of the intake motor.  If front intake, sets both the intake and
   * transfer motors to this perecent speeed.
   * @param percent  -1 to +1  (+ = intake, - = outtake)
   * @param intake  Intake subsystem
   * @param log
   */
  public IntakeSetPercentOutput(double percent, Intake intake, FileLog log) {
    this(percent, percent, intake, log);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, intake.getName(), "SetPercentOutput", "Intake Percent", intakePercent,
      "Transfer Percent", transferPercent);

    if (intake instanceof IntakeFront) {
      ((IntakeFront)intake).setMotorPercentOutput(intakePercent, transferPercent);
    } else {
      intake.setMotorPercentOutput(intakePercent);
    }
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
