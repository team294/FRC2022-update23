// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.FeederSetPercentOutput;
import frc.robot.commands.FeederStop;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeederBallReadyToShoot extends SequentialCommandGroup {
  
  /**
   * Runs the feeder until a ball is sensed in the feeder.
   * @param feeder feeder subsytem
   * @param log logfile
   */
  public FeederBallReadyToShoot(Feeder feeder, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new FeederSetPercentOutput(0.3, feeder, log),
        new WaitCommand(10).withInterrupt(feeder::isBallPresent),
        new FeederStop(feeder, log)
    );
  }
}
