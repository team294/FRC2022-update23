package frc.robot.commands.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.TargetType;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;


public class AutoFiveBall extends SequentialCommandGroup {

/**
 * Five ball trajectory starting from right side
 * 
 * @param waitTime seconds to wait before starting
 * @param trajectoryCache cached trajectories
 * @param driveTrain drivetrain subsystem
 * @param shooter shooter subsystem
 * @param feeder feeder subsystem
 * @param intake intake subsystem
 * @param uptake update subsystem
 * @param limelight front limelight for driving with vision
 * @param log file logger
 */
public AutoFiveBall(double waitTime, TrajectoryCache trajectoryCache, DriveTrain driveTrain, Shooter shooter, Feeder feeder, Intake intake, Uptake uptake, LimeLight limeLight, FileLog log) {
    addCommands(
      new FileLogWrite(false, false, "AutoFiveBall", "starting", log),
      new WaitCommand(waitTime),                                        
      
      new DriveFollowTrajectory(CoordType.kAbsoluteResetPose, StopType.kBrake, trajectoryCache.cache[TrajectoryType.rightStartToRightBall.value], driveTrain, log),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.rightShoot.value], driveTrain, log),
      new DriveTurnGyro(TargetType.kAbsolute, -90, 150, 200, 5, driveTrain, limeLight, log),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.rightToCenter.value], driveTrain, log),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.centerToBack.value], driveTrain, log),
      new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.backToCenter.value], driveTrain, log),
      new DriveTurnGyro(TargetType.kAbsolute, 90, 150, 200, 5, driveTrain, limeLight, log),
      
      new FileLogWrite(false, false, "AutoFiveBall", "end", log)
    );

  }
}
