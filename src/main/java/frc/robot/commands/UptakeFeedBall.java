package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

public class UptakeFeedBall extends CommandBase {
  private Uptake uptake;
  private Feeder feeder;
  private FileLog log;

  /**
   * Feeds a ball from the uptake into the feeder
   * 
   * @param uptake uptake subsystem
   * @param log file logger
   */
  public UptakeFeedBall(Uptake uptake, Feeder feeder, FileLog log) {
    this.uptake = uptake;
    this.log = log;
    this.feeder = feeder;
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "UptakeFeedBall", "Init");
 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log.writeLog(false, "UptakeFeedBall", "Execute"); 
    uptake.setEjectPercentOutput(-0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "UptakeFeedBall", "End", "Interrupted", interrupted);
    uptake.setEjectPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(feeder.isBallPresent()){
      log.writeLog(false, "UptakeFeedBall", "Detected ball in feeder");
      return true;
    }
    else return false;
  }
}
