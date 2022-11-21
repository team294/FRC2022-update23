package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;


public class AutoBeachBlitzThreeBall extends SequentialCommandGroup {

/**
 * Shoot preloaded ball and taxi out of the tarmac
 * 
 * @param waitTime seconds to wait before starting
 * @param driveTrain drivetrain subsystem
 * @param shooter shooter subsystem
 * @param feeder feeder subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param limelight front limelight for driving with vision
 * @param log file logger
 */
public AutoBeachBlitzThreeBall(double waitTime, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, Turret turret, PiVisionHub pivisionhub, LimeLight limeLight, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoBeachBlitzThreeBall", "starting", log),
      new WaitCommand(waitTime),                        // delay from shuffleboard
      new DriveZeroGyro(0, driveTrain, log),
      new PiVisionHubSetLEDState(1, pivisionhub),      

      new DriveResetPose(0, 0, 0, driveTrain, log),   // set initial pose to 180 degrees away from the ball (facing the hub)
      new PiVisionHubSetLEDState(1, pivisionhub),       // turn on led light for vision

      // Back up so we can see the target for the first shot
      new DriveStraight(-AutoConstants.driveFromStartToSeeHub, TargetType.kAbsolute, 0, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),

      // shoot
      new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log).withTimeout(3), 
      new ShootSetup(true, AutoConstants.ballOneRPM, pivisionhub, shooter, log),
      new ShootSequence(uptake, feeder, shooter, log),

      // turn to third ball
      new DriveTurnGyro(TargetType.kAbsolute, 180, 300, 200, 3, driveTrain, limeLight, log).withTimeout(2),

      // get ready to intake ball
      new IntakePistonSetPosition(true, intake, log),
      new IntakeToColorSensor(intake, uptake, log),

      // drive to third ball
      new DriveStraight(AutoConstants.driveToBallTwoInMeters-AutoConstants.driveFromStartToSeeHub, TargetType.kAbsolute, 
                        180, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),

      // turn back to hub
      new DriveTurnGyro(TargetType.kAbsolute, 0, 300, 200, 3, driveTrain, limeLight, log).withTimeout(2),

      // shoot 
      new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log).withTimeout(3),       // F2 -- Added 3-sec timeout, in case we lose vision
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(uptake, feeder, shooter, log),

      // Prep for Teleop mode
      new IntakeToColorSensor(intake, uptake, log),           // F8 added IntakeToColorSensor, since it was removed from ShootSequence in F7

      new FileLogWrite(false, false, "AutoBeachBlitzThreeBall", "end", log)

    );
  }
}
