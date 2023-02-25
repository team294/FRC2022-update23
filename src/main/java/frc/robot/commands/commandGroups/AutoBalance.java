package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;


public class AutoBalance extends SequentialCommandGroup {

/**
 * Drive backwards and balance
 * 
 * @param waitTime seconds to wait before starting
 * @param driveTrain drivetrain subsystem
 * @param log file logger
 */
public AutoBalance(double waitTime, DriveTrain driveTrain, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoBalance", "starting", log),

      new DriveZeroGyro(0, driveTrain, log),  

      // drive backwards over charging station
      // new DriveStraight(-2.0, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, null, log).withTimeout(5),

      // drive forwards and stop when balanced
      new Balance(4.0, TargetType.kRelative, 0.0, 0.5, 3.8, true, driveTrain, null, log).withTimeout(10),

      new FileLogWrite(false, false, "AutoBalance", "end", log)

    );
  }
}
