package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class IntakeToColorSensor extends SequentialCommandGroup {

/**
 * Turn on intake and uptake to intake a ball up to the color sensor and hold
 * 
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param log file logger
 */
public IntakeToColorSensor(Intake intake, Uptake uptake, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "IntakeToColorSensor", "starting", log),
      new ConditionalCommand(
        sequence(
          new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intake, log),
          new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)
        ),
        new FileLogWrite(false, false, "IntakeToColorSensor", "intake not extended", log),
        // () -> true
        () -> intake.getPistonExtended()
        ),
        new FileLogWrite(false, false, "IntakeToColorSensor", "end", log)
    );
  }
}
