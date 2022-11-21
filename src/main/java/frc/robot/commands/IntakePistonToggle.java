package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.commandGroups.IntakeToColorSensor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class IntakePistonToggle extends CommandBase {
  private Intake intake;
  private Uptake uptake;
  private FileLog log;
  /**
   * Toggles the intake between deployed and retracted 
   * 
   * @param intake intake subsystem
   * @param log
   */
  public IntakePistonToggle(Intake intake, Uptake uptake, FileLog log) {
    this.intake = intake;
    this.uptake = uptake;
    this.log = log;
    addRequirements(intake, uptake);
    
  }
   
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    log.writeLog(false, intake.getName(), "Toggle", "Extended at start", intake.getPistonExtended());

    // turn off/on intake before closing/opening intake
    if (intake.getPistonExtended()) {
      intake.setMotorPercentOutput(0);
      uptake.setUptakePercentOutput(0);
      intake.setPistonExtended(false);
    } else {
      intake.setPistonExtended(true);
      CommandScheduler.getInstance().schedule(new IntakeToColorSensor(intake, uptake, log));
    }

    // temp fix for intake
    // intake.setPistonExtended(true);
    // CommandScheduler.getInstance().schedule(new IntakeToColorSensor(intake, uptake, log));
    
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
