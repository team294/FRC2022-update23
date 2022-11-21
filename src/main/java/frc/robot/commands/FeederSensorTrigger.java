package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class FeederSensorTrigger extends CommandBase {
  Feeder feeder;
  FileLog log;

  /**
   * Called when feeder sensor is triggered 
   * 
   * @param uptake subsystem
   * @param log file logger
   */
  public FeederSensorTrigger(Feeder feeder, FileLog log) {
    this.feeder = feeder;
    this.log = log;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, "FeederSensorTrigger", "Init");
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
