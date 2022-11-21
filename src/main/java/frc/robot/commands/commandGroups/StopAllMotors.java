package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;


public class StopAllMotors extends SequentialCommandGroup {

/**
 * Stop all motors
 * 
 * @param feeder feeder subsystem
 * @param shooter shooter subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param log file logger
 */
public StopAllMotors(Feeder feeder, Shooter shooter, Intake intake, Uptake uptake, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "StopAllMotors", "starting", log),
      parallel(
        new IntakeSetPercentOutput(0, 0, intake, log),
        new UptakeSetPercentOutput(0, 0, uptake, log),
        new FeederStop(feeder, log),
        new ShooterStop(shooter, log)
      ),
      new FileLogWrite(false, false, "StopAllMotors", "end", log)
    );
  }
}
