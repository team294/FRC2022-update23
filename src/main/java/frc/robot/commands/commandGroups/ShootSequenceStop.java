package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ShootSequenceStop extends SequentialCommandGroup {
  /**
   * Turn off motors after the shoot sequence to avoid inadvertent shots
   * 
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param log
   */
  public ShootSequenceStop(Uptake uptake, Feeder feeder, FileLog log) {
    addCommands(
      new FileLogWrite(true, false, "ShootSequenceStop", "start", log),
      new FeederSetPercentOutput(0, feeder, log),                           // turn off the feeder
      new EjectStop(uptake),                                                // turn off the eject wheel
      new FileLogWrite(true, false, "ShootSequenceStop", "end", log)
    );
  }
}
