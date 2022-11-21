// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakePistonSetPosition;
import frc.robot.commands.IntakeSetPercentOutput;
import frc.robot.commands.UptakeSetPercentOutput;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.FileLog;


public class IntakeToggle extends SequentialCommandGroup {
  /**
   * If the intake is extended, retract and stop motors, otherwise extend and start motors
   * @param intake intake subsytem
   * @param uptake uptake subsystem
   * @param log 
   */
  public IntakeToggle(Intake intake, Uptake uptake, FileLog log) {
    addCommands(
      new ConditionalCommand(
        sequence(
          // Retract intake
          new IntakePistonSetPosition(false, intake, log),
          new WaitCommand(0.5),
          new IntakeSetPercentOutput(0, intake, log),
          new UptakeSetPercentOutput(0, 0, uptake, log)
        ), 
        sequence(
          // Extend intake
          new IntakePistonSetPosition(true, intake, log),
          new IntakeToColorSensor(intake, uptake, log)
        ), 
      () -> intake.getPistonExtended())
    );
  }
}
