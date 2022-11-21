// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeFront;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRetractAndFlush extends SequentialCommandGroup {

  /**
   * Retracts the intake and turns off the intake motors.
   * If there are less than 2 balls in the uptake/feeder, then runs a "flush" sequence
   * on the intake/transfer motors to try to knock any balls out of the dead space in 
   * the transfer area.
   * @param intakeFront intake subsystem
   * @param uptake uptake subsystem (not a requirement, only used for sensors)
   * @param feeder feeder subsystem (not a requirement, only used for sensors)
   * @param log
   */
  public IntakeRetractAndFlush(IntakeFront intakeFront, Uptake uptake, Feeder feeder, FileLog log) {

    addCommands(
      new IntakeRetractAndTurnOffMotors(false, intakeFront, log),

      new ConditionalCommand(
        sequence(
          new IntakeSetPercentOutput(-IntakeConstants.onPct, IntakeConstants.onPctTransfer, intakeFront, log), // turn on transfer wheels to clear jams, intake in reverse to clear jams (F7)
          new WaitCommand(0.5),
          new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intakeFront, log), // turn on transfer wheels to clear jams
          new WaitCommand(1.0),
          new IntakeSetPercentOutput(0, 0, intakeFront, log)
        ),
        new WaitCommand(0.01), 
        () -> !((uptake.isBallGoingToFeeder() && uptake.isBallAtColorSensor()) || (uptake.isBallGoingToFeeder() && feeder.isBallPresent()) || (uptake.isBallAtColorSensor() && feeder.isBallPresent()))
        )

    );
  }
}
