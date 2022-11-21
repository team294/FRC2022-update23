// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.buildString;
import static frc.robot.Constants.*;

public class ShooterSetPIDSV extends CommandBase {

  private FileLog log;
  private Shooter shooter;
  private boolean fromShuffleboard;
  private double P, I, D, S, V;

  /**
   * Sets PIDSV parameters for motor velocity control from Shuffleboard.
   * 
   * <p> For Feedback terms (P, I, D), the velocity error term is in units [encoder ticks]/[100ms].  The output drive is on a 1023 scale.
   * Note that RPM / ShooterConstants.rawVelocityToRPM will convert to [encoder ticks]/[100ms].
   * 
   * <p> For Feedforward terms, the velocity is in RPM.  The output drive is on a 1.0 scale.
   * 
   * <p> P = Proportional feedback term, in [1023-scale-output]/[ [encoder ticks]/[100ms] ]
   * <p> I = Integral feedback term.  Use a small IZone and start the I-gain at something small (0.001).
   * <p> D = Derivative feedback term.  D-gain can start typically at 10 X P-gain.
   * <p> S = Feedforward offset, in percent output (-1 to +1)
   * <p> V = Feedforward velocity term, in [percent output (-1 to +1)]/[RPM]
   * @param shooter Shooter subsystem to set PIDF parameters
   * @param log File for logging
   */
  public ShooterSetPIDSV(Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.fromShuffleboard = true;
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber(buildString(shooter.getName(), " P"), ShooterConstants.kP);
    SmartDashboard.putNumber(buildString(shooter.getName(), " I"), ShooterConstants.kI);
    SmartDashboard.putNumber(buildString(shooter.getName(), " D"), ShooterConstants.kD);
    SmartDashboard.putNumber(buildString(shooter.getName(), " S"), ShooterConstants.kS);
    SmartDashboard.putNumber(buildString(shooter.getName(), " V"), ShooterConstants.kV);
  }

  /**
   * Sets PIDSV parameters for motor velocity control.
   * 
   * <p> For Feedback terms (P, I, D), the velocity error term is in units [encoder ticks]/[100ms].  The output drive is on a 1023 scale.
   * Note that RPM / ShooterConstants.rawVelocityToRPM will convert to [encoder ticks]/[100ms].
   * 
   * <p> For Feedforward terms, the velocity is in RPM.  The output drive is on a 1.0 scale.
   * @param P Proportional feedback term, in [1023-scale-output]/[ [encoder ticks]/[100ms] ]
   * @param I Integral feedback term.  Use a small IZone and start the I-gain at something small (0.001).
   * @param D Derivative feedback term.  D-gain can start typically at 10 X P-gain.
   * @param S Feedforward offset, in percent output (-1 to +1)
   * @param V Feedforward velocity term, in [percent output (-1 to +1)]/[RPM]
   * @param shooter Motor subsystem to set PIDF parameters
   * @param log File for logging
   */
  public ShooterSetPIDSV(double P, double I, double D, double S, double V, Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;
    this.fromShuffleboard = false;
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) {
      // Get PID coefficients from SmartDashboard
      P = SmartDashboard.getNumber(buildString(shooter.getName(), " P"), ShooterConstants.kP);
      I = SmartDashboard.getNumber(buildString(shooter.getName(), " I"), ShooterConstants.kI);
      D = SmartDashboard.getNumber(buildString(shooter.getName(), " D"), ShooterConstants.kD);
      S = SmartDashboard.getNumber(buildString(shooter.getName(), " S"), ShooterConstants.kS);
      V = SmartDashboard.getNumber(buildString(shooter.getName(), " V"), ShooterConstants.kV);
    } else {
      SmartDashboard.putNumber(buildString(shooter.getName(), " P"), P);
      SmartDashboard.putNumber(buildString(shooter.getName(), " I"), I);
      SmartDashboard.putNumber(buildString(shooter.getName(), " D"), D);
      SmartDashboard.putNumber(buildString(shooter.getName(), " S"), S);
      SmartDashboard.putNumber(buildString(shooter.getName(), " V"), V);
    }

    log.writeLog(false, shooter.getName(), "Set PIDF", "P", P, "I", I, "D", D, "S", S, "V", V);
    shooter.setPIDSV(P, I, D, S, V);
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
