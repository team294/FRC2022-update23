// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Uptake;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.BallLocation;
import frc.robot.commands.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeBall extends SequentialCommandGroup {
  /** Creates a new IntakeBall. */
  public IntakeBall(BallColor teamColor, Intake intake, Uptake uptake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeSetPercentOutput(0.25, intake, log)
      .perpetually().withInterrupt(uptake::isBallAtColorSensor), 
      new SetBallCount(1, BallLocation.kUptake, log),
      new ConditionalCommand( 
        new IntakeStop(intake, log),
        new WaitCommand(0.2),
        () -> BallCount.getTotalBallCount() == 2
      ),
      new ConditionalCommand(
        new ConditionalCommand(
          sequence(
          new UptakeSetPercentOutput(0.25, true, uptake, log), 
          new WaitCommand(2),
          new UptakeStop(uptake, log)
          ),
          new WaitCommand(0.2),
         () -> BallCount.getBallCount(BallLocation.kFeeder) == 0
      ),
        sequence(
          new UptakeSetPercentOutput(0.25, false, uptake, log),
          new WaitCommand(2),
          new SetBallCount(0, BallLocation.kUptake, log),
          new SetBallCount(1, BallLocation.kFeeder, log)
        ),
      () -> uptake.getBallColor() == teamColor
      )
    );
  }
}
