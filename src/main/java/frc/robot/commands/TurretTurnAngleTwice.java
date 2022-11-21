// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.PiVisionHub;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretTurnAngleTwice extends SequentialCommandGroup {
  /** Creates a new TurretTurnAngleTwice. */
  public TurretTurnAngleTwice(TargetType type, boolean regenerate, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

      new TurretTurnAngle(type, regenerate, turret, piVisionHub, log).withTimeout(0.9), //uses values from Shuffleboard
      new TurretTurnAngle(type, regenerate, turret, piVisionHub, log).withTimeout(0.9)
      
      
      
    );
  }
}
