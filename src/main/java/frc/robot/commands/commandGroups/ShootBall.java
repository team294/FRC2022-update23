package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.Constants.BallLocation;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class ShootBall extends SequentialCommandGroup {
  /** Creates a new ShootBall. */
  public ShootBall(Shooter shooter, Uptake uptake, Feeder feeder, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterSetVelocity(InputMode.kDistFeet, shooter, log),      // Use distance from Shuffleboard
      new FeederSetPercentOutput(feeder, log),
      new WaitCommand(2),
      new UptakeSetPercentOutput(0.25, false, uptake, log),
      new WaitCommand(3),
      new SetBallCount(0, BallLocation.kFeeder, log),
      new SetBallCount(0, BallLocation.kUptake, log),
      new ShooterStop(shooter, log),
      new FeederStop(feeder, log),
      new EjectStop(uptake)
    );
  }
}
