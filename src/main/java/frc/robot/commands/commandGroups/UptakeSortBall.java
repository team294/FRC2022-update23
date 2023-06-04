package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

public class UptakeSortBall extends SequentialCommandGroup {

  /**
   * Sequence to handle a ball once it hits the color sensor in the uptake
   * Ejects the ball if it matches the ejectColor.
   * If this is the first ball, then advances the ball to the feeder.
   * If this is the 2nd ball, then turns off the uptake motor and closes the intake.
   * Vibrates the X-box controller once or twice, depending on the number of balls in the uptake and feeder.
   * 
   * @param intake intake subsystem (not a required subsystem)
   * @param uptake uptake subsystem
   * @param feeder feeder subsystem
   * @param xboxController XBox controller to use for rumbling
   * @param log logger
   */
  public UptakeSortBall(Intake intake, Uptake uptake, Feeder feeder, /*CommandXboxController xboxController,*/ FileLog log) {

    addCommands(
      new FileLogWrite(true, false, "UptakeSortBall", "start sequence", log),
      new LogEnableFastLogging(true, uptake, log),

      new FeederSetPercentOutput(0, feeder, log),   // turn off the feeder just in case so we don't shoot while intaking
      new WaitCommand(0.15),       // Wait more than 100ms so that the color sensor stabilizes

      new ConditionalCommand(
        // if it is the wrong color, eject the ball
        Commands.parallel(
          new FileLogWrite(true, false, "UptakeSortBall", "eject", log),
          new UptakeEjectBall(uptake, log).withTimeout(1)
        ),
        // if it is the right color then check if there is room in the feeder
        new ConditionalCommand(
          // if there is nothing in the feeder then feed it
          Commands.parallel(
            new FileLogWrite(true, false, "UptakeSortBall", "send to feeder", log),
            // new XboxRumble(0.5, 0.25, 1, xboxController, log),    // Notify drive that the robot has one ball
            new UptakeFeedBall(uptake, feeder, log).withTimeout(1)
            //new UptakeToFeeder(uptake, feeder, log).withTimeout(1)
          ),
          // if there is something in the feeder, turn off the uptake and hold the 2nd ball in the uptake and close intake
          Commands.parallel(
            new FileLogWrite(true, false, "UptakeSortBall", "hold in uptake", log),
            // new XboxRumble(0.5, 0.25, 2, xboxController, log),    // Notify drive that the robot has two balls
            // this logic has moved to the end of command group to allow sensors to stabilize
            //new UptakeStop(uptake, log)

            // Close intake.  Schedule the intake command separately, so UptakeSortBall does not require the Intake 
            // subsystem.  This prevents conflicts between UptakeSortBall and the intake extend/retract buttons.
            new ScheduleCommand(new IntakeRetractAndTurnOffMotors(true, intake, log))
          ),
          () -> !feeder.isBallPresent()
        ),
      () -> (uptake.getEjectColor() != BallColor.kNone) && (uptake.getBallColor() == uptake.getEjectColor())
      ),

      new LogEnableFastLogging(false, uptake, log),
      // new WaitCommand(0.5),

      // turn off uptake if we have balls in feeder and uptake, otherwise turn it on
      new ConditionalCommand(
          new UptakeSetPercentOutput(0, 0, uptake, log),
          new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log), 
        () -> feeder.isBallPresent() && uptake.isBallAtColorSensor()),

      new FileLogWrite(true, false, "UptakeSortBall", "end sequence", log)
    );
  }
}


