// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRetractAndTurnOffMotors extends SequentialCommandGroup {
  /**
   * 
   * @param flush  True = reverse intake and flush any 3rd ball before retracting intake.  False = immediately retract
   * @param intake
   * @param log
   */
  public IntakeRetractAndTurnOffMotors(boolean flush, Intake intake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (flush) {
      // Flush 3rd ball from intake, then close
      addCommands(
        new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPctTransfer, intake, log),
        new WaitCommand(1.25),
        new IntakePistonSetPosition(false, intake, log),
        new IntakeSetPercentOutput(0, intake, log)
      );
    } else {
      // Immediately close
      addCommands(
        new IntakePistonSetPosition(false, intake, log),
        new WaitCommand(0.5),
        new IntakeSetPercentOutput(0, intake, log)
      );
    }
  }
}
