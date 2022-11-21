package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;

/**
 * Set shooter velocity with vision or set point
 * @param useVision true if vision should be used for determining velocity
 * @param velocity target rpm of the shooter if not using vision or if vision can't see target
 * @param vision vision subsystem to use for distance. Set to null when not using vision.
 * @param shooter shooter subsystem
 * @param log log subsystem
 */
public class ShootSetup extends SequentialCommandGroup {

  public ShootSetup(boolean useVision, double velocity, PiVisionHub vision, Shooter shooter, FileLog log) {

    addCommands(
      sequence(
        new FileLogWrite(false, false, "ShootSetup", "Setup", log, "Velocity", velocity, "useVision", useVision ),
        new ConditionalCommand(
          // if get velocity from vision
          parallel (
            new FileLogWrite(false, false, "ShootSetup", "SetupWithVision", log, "Velocity", velocity, "Vision", vision == null ? "0":vision.getDistance()),
            new ShooterSetVelocity(shooter, vision, log)
          ),
          // use velocity without vision
          parallel (
            new FileLogWrite(false, false, "ShootSetup", "SetupWithVelocity", log, "velocity", velocity),
            new ShooterSetVelocity(InputMode.kSpeedRPM, velocity, shooter, log)
          ),
          () -> (useVision && vision != null)
        ),
        new FileLogWrite(false, false, "ShootSetup", "End", log, "Velocity", velocity, "useVision", useVision,"visionDistance", vision == null ? "null vision" : vision.getDistance())
      )
    );
  }
}
