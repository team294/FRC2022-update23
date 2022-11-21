// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeExtendAndTurnOnMotors extends SequentialCommandGroup {

  /**
   * Extends intake, turns on intake motor and transfer motor.
   * If there is not a ball at the color sensor, then turns on the uptake motor (no requires)
   * @param intake intake subsystem
   * @param uptake uptake subsystem (does not require this subsystem)
   * @param log
   */
  public IntakeExtendAndTurnOnMotors(Intake intake, Uptake uptake, FileLog log) {

    addCommands(
      new IntakePistonSetPosition(true, intake, log),
      // new IntakeToColorSensor(intake, uptake, log)
      new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intake, log),

      // Turn on uptake, unless we are holding a ball at the color sensor
      new ConditionalCommand(
        // No ball at color sensor
        new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log, false),
        // Ball at color sensor
        new InstantCommand(),
        () -> !uptake.isBallAtColorSensor()
      )
    );
  }
}
