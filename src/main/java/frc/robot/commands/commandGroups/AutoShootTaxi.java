package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;


public class AutoShootTaxi extends SequentialCommandGroup {

/**
 * Shoot preloaded ball and taxi out of the tarmac
 * 
 * @param waitTime seconds to wait before starting
 * @param driveTrain drivetrain subsystem
 * @param shooter shooter subsystem
 * @param feeder feeder subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param vision pivision subsystem
 * @param limelight front limelight for driving with vision
 * @param log file logger
 */
public AutoShootTaxi(double waitTime, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, Turret turret, PiVisionHub pivisionhub,LimeLight limeLight, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoShootTaxi", "starting", log),
      new WaitCommand(waitTime),                        // delay from shuffleboard
      new DriveZeroGyro(0, driveTrain, log),  

      // drive out backwards so vision can see
      new DriveStraight(-AutoConstants.driveToBallTwoInMeters, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log).withTimeout(2),

      // target with vision
      new PiVisionHubSetLEDState(1, pivisionhub), 
      new TurretTurnAngle(TargetType.kVisionOnScreen, 0, 3, turret, pivisionhub, log).withTimeout(2.0),

      // shoot
      new ShootSetup(true, AutoConstants.ballOneRPM, pivisionhub, shooter, log),
      new ShootSequence(uptake, feeder, shooter, log),
    
      // deploy intake so we are ready to go in teleop
      new IntakePistonSetPosition(true, intake, log),

      // turn on intake
      new IntakeToColorSensor(intake, uptake, log),

      new FileLogWrite(false, false, "AutoShootTaxi", "end", log)

    );
  }
}
