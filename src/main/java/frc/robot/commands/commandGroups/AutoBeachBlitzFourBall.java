package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetType;
import frc.robot.Constants.UptakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;


public class AutoBeachBlitzFourBall extends SequentialCommandGroup {

/**
 * Four ball auto starting in the center facing the ball with hub directly behind lined up for a shot
 * 
 * @param driveTrain drivetrain subsystem
 * @param shooter shooter subsystem
 * @param feeder feeder subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param limelight front limelight for driving with vision
 * @param log file logger
 */
public AutoBeachBlitzFourBall(DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, Turret turret, TrajectoryCache trajectoryCache, PiVisionHub pivisionhub, LimeLight limeLight, FileLog log) {
    addCommands(
      // setup
      new FileLogWrite(false, false, "AutoBeachBlitzFourBall", "starting", log),
      new DriveResetPose(0, 0, 0, driveTrain, log),   // set initial pose to 0 degrees away from the hub (facing the hub)
      new PiVisionHubSetLEDState(1, pivisionhub),       // turn on led light for vision

      // Shoot first two balls
      new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log),
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(uptake, feeder, shooter, log),
      
      // Turn to third ball
      new DriveTurnGyro(TargetType.kAbsolute, 180, 300, 200, 3, driveTrain, limeLight, log).withTimeout(2),
      
      // get ready to intake ball
      new IntakePistonSetPosition(true, intake, log),
      new IntakeToColorSensor(intake, uptake, log),

      // drive to third ball
      new DriveStraight(AutoConstants.driveToBallTwoInMeters, TargetType.kAbsolute, 180, 2.66, 3.8, true, driveTrain, limeLight, log).withTimeout(3),

      // go to fourth ball
      new DriveTurnGyro(TargetType.kAbsolute, 168, 300, 200, 6, driveTrain, limeLight, log).withTimeout(3),
      new DriveStraight(3.8, TargetType.kAbsolute, 168, 3.5, 3.8, true, driveTrain, limeLight, log).withTimeout(3),  // Reduced 4.2 to 4.0 for F1 to 3.8 for F2, max vel 2.66 to 3.5

      parallel(
        // retract intake
        new IntakePistonSetPosition(false, intake, log),

        // drive back to sweet spot
        //new DriveTurnGyro(TargetType.kAbsolute, 0, 300, 200, 3, driveTrain, limeLight, log).withTimeout(3),
        new DriveStraight(-3.8, TargetType.kAbsolute, 168, 3.5, 3.8, true, driveTrain, limeLight, log).withTimeout(3)  // F1 max vel 2.66 to 3.5
      ),

      // turn to face hub
      parallel(
        new DriveTurnGyro(TargetType.kAbsolute, 75, 300, 200, 6, driveTrain, limeLight, log).withTimeout(2),      // F3 change from 105 to 75 deg
        new TurretTurnAngle(TargetType.kAbsolute, 70, 1, turret, pivisionhub, log).withTimeout(2)             // F5 change from 60 to 70
      ),
      new TurretTurnAngle(TargetType.kVisionScanRight, 0, 1, turret, pivisionhub, log).withTimeout(2),      // F3 add scan after turn is finished

      // shoot
      new ShootSetup(true, AutoConstants.ballTwoRPM, pivisionhub, shooter, log),
      new ShootSequence(uptake, feeder, shooter, log),

      // Prep for Teleop mode
      new IntakeToColorSensor(intake, uptake, log),           // F8 added IntakeToColorSensor, since it was removed from ShootSequence in F7

      new FileLogWrite(false, false, "AutoBeachBlitzFourBall", "end", log)

    );
  }
}
