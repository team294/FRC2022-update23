package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;


public class AutoTaxi extends SequentialCommandGroup {

/**
 * Taxi out of the tarmac in reverse
 * 
 * @param waitTime seconds to wait before starting
 * @param limelight front limelight for driving with vision
 * @param driveTrain drivetrain subsystem
 * @param log file log
 * 
 */
public AutoTaxi(double waitTime, LimeLight limeLight, Intake intake, Uptake uptake, DriveTrain driveTrain, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoTaxi", "starting", log),
      new WaitCommand(waitTime),                                        

      new DriveZeroGyro(0, driveTrain, log),      
      

      // drive out backwards so we are ready to shoot preloaded ball
      new DriveStraight(-AutoConstants.driveToBallTwoInMeters, TargetType.kRelative, 0.0, 2.61, 3.8, true, driveTrain, limeLight, log).withTimeout(2),

      // deploy intake so we are ready to go in teleop
      new IntakePistonSetPosition(true, intake, log),

      new IntakeToColorSensor(intake, uptake, log),
      
      new FileLogWrite(false, false, "AutoTaxi", "end", log)
    );
  }
}
