package frc.robot.commands.commandGroups;


import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ShootSequence extends SequentialCommandGroup {
  /**
   * Shoot sequence triggers that action of shooting all the balls by running the feeder and uptake.
   * The aiming of the turret and the speed of the shooter should be set before this sequence is run.
   * 
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param shooter shooter subsystem
   * @param log
   */
  public ShootSequence(Uptake uptake, Feeder feeder, Shooter shooter, FileLog log) {
    addCommands(
      new FileLogWrite(true, false, "ShootSequence", "shooting", log),
      new ShooterSetVelocity(InputMode.kLastSetSpeed, shooter, log).withTimeout(1),         // Wait for shooter to be at speed
      new FeederSetPercentOutput(FeederConstants.onPct, feeder, log),                       // turn on feeder to send first ball to shooter
      new WaitCommand(2).until(() -> !feeder.isBallPresent()),                      // turn off feeder when ball clears feeder
      new FeederSetPercentOutput(0, feeder, log),                                           // turn off feeder
      new WaitCommand(0.2),                                                                 // Give first ball a little time to clear feeder

      new ConditionalCommand(
        // There is a 2nd ball in the uptake
        new ConditionalCommand(
          new SequentialCommandGroup(
            // Ball is wrong color = eject
            new UptakeEjectBall(uptake, log).withTimeout(1),
            new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)               // Turn off eject motor, to prepare for next ball
          ),
          new SequentialCommandGroup(
            // Ball is right color = shoot
            new UptakeSetPercentOutput(UptakeConstants.onPct, false, uptake, log),          // make sure uptake is running to send second ball to feeder
            new ShooterSetVelocity(InputMode.kLastSetSpeed, shooter, log).withTimeout(1),   // Wait for shooter to be at speed
            new WaitCommand(0.5).until(feeder :: isBallPresent),
            new FeederSetPercentOutput(FeederConstants.onPct, feeder, log),                 // turn on feeder to send second ball to shooter
            new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log),              // Turn off eject motor, to prepare for next ball
            new WaitCommand(1).until(() -> !feeder.isBallPresent()),                // wait for second ball to shoot 
            new WaitCommand(.15)                                                            // Wait for ball to exit shooter
          ), 
          () -> (uptake.getEjectColor() != BallColor.kNone) && (uptake.getBallColor() == uptake.getEjectColor())
        ),
        // There is not a second ball in the uptake
        new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log),                  // Turn off eject motor, to prepare for next ball
        () -> uptake.isBallAtColorSensor() || uptake.isBallGoingToFeeder()
      ),
      
      new FeederSetPercentOutput(0, feeder, log),                                           // turn off the feeder

      new FileLogWrite(true, false, "ShootSequence", "end", log)
    );
  }
}