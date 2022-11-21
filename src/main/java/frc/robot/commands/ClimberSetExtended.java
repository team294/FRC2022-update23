package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.utilities.FileLog;

public class ClimberSetExtended extends CommandBase {
  /** Creates a new testUptake. */
  private final Climber climber;
  private FileLog log;
  private boolean extended;

  public ClimberSetExtended(boolean extended, Climber climber, FileLog log) {
    this.climber = climber;
    this.log = log;
    this.extended = extended;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.setPistonExtended(extended);
    log.writeLog(false, "Climber", "initialize", "SetExtended", extended);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
