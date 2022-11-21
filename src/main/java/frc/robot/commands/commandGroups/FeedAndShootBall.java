// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedAndShootBall extends SequentialCommandGroup {
  /** Creates a new FeedAndShootBall. */
  
  public FeedAndShootBall(Shooter shooter, Feeder feeder, Uptake uptake, FileLog log, BallColor teamColor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      parallel(
        new ShooterSetVelocity(InputMode.kSpeedRPM, 3000, shooter, log),
        new ConditionalCommand(
          new WaitCommand(0.02), 
          sequence(
            parallel(  
              new FeederSetPercentOutput(.3, feeder, log),
              sequence( 
                new ConditionalCommand(
                  new WaitCommand(0.02),
                  new UptakeSetPercentOutput(0.25, false, uptake, log)
                  .perpetually().withInterrupt(uptake::isBallAtColorSensor), 
                  () -> uptake.isBallAtColorSensor()
                ),
                new ConditionalCommand(
                  // Load ball to feeder
                  new UptakeSetPercentOutput(0.25, true, uptake, log), 
                  // Eject ball
                  new UptakeSetPercentOutput(0.25, false, uptake, log), 
                  () -> uptake.getBallColor() == teamColor
                )
              ) 
            ),
            new WaitCommand(5).withInterrupt(feeder::isBallPresent),
            new UptakeStop(uptake, log),
            new ConditionalCommand(
              new WaitCommand(.02), 
              new FeederStop(feeder, log),
              () -> Math.abs(shooter.getVelocityPIDError()) < ShooterConstants.pidErrorTolerance
            )
          ), 
          () -> feeder.isBallPresent()
        )
      ),
      new FeederSetPercentOutput(.3, feeder, log),
      new WaitCommand(2),
      new ShooterStop(shooter, log),
      new FeederStop(feeder, log)
      );
  }

}
