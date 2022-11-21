// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

/**
 * Class that defines and caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
public class TrajectoryCache {
    private FileLog log;
   
    private static int trajectoryCount = 13;
    public Trajectory[] cache = new Trajectory[trajectoryCount];

    public enum TrajectoryType {
        test(0),
        testCurve(1),
        taxi(2),
        firstBall(3),
        rightStartToRightBall(4),
        rightToCenter(5),
        centerToBack(6),
        backToCenter(7),
        rightShoot(8),
        centertoCenterBall(9),
        leftToLeftBall(10),
        centerBallToBackFourball(11),
        backToCenterFourBall(12);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TrajectoryType(int value) { this.value = value; }
    }
	
    /**
     * Build all trajectories in this.cache[] for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;

        cache[TrajectoryType.test.value] = calcTrajectory("Test", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(6.0, 0, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.testCurve.value] = calcTrajectory("Test Curve", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(3, 3, new Rotation2d(Math.toRadians(90.0)))
        );        

        cache[TrajectoryType.taxi.value] = calcTrajectory("Taxi", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(1.3, 0, new Rotation2d(Math.toRadians(0.0)))
        );

        cache[TrajectoryType.firstBall.value] = calcTrajectory("First Ball", 0.4, 0.4, false, 
            new Pose2d(0, 0, new Rotation2d(0.0)),
            List.of(),
            new Pose2d(1.3, 0, new Rotation2d(Math.toRadians(0.0)))
        );        

        cache[TrajectoryType.rightStartToRightBall.value] = calcTrajectory("RightStartToRightBall", 0.4, 0.4, true, 
            new Pose2d(0.5, 7.600, new Rotation2d(Math.toRadians(180))),
            List.of(
            ),
            new Pose2d(1.8, 7.600, new Rotation2d(Math.toRadians(180)))
        );

        cache[TrajectoryType.rightShoot.value] = calcTrajectory("Right Shoot", 0.4, 0.4, false,
            new Pose2d(1.8, 7.600, new Rotation2d(Math.toRadians(180))),
            List.of(
            ),
            new Pose2d(0.5, 7.600, new Rotation2d(Math.toRadians(180)))
        );

        cache[TrajectoryType.rightToCenter.value] = calcTrajectory("Right To Center", 0.4, 0.4, false,
            new Pose2d(0.5, 7.600, new Rotation2d(Math.toRadians(-90))),
            List.of(
            ),
            new Pose2d(0.438, 5.080, new Rotation2d(Math.toRadians(-90)))
        );

        cache[TrajectoryType.centerToBack.value] = calcTrajectory("Center To Back", 0.4, 0.4, false,
            new Pose2d(0.438, 5.080, new Rotation2d(Math.toRadians(-90))),
            List.of(
            ),
            new Pose2d(1.100, 1.200, new Rotation2d(Math.toRadians(-45)))
        );

        cache[TrajectoryType.backToCenter.value] = calcTrajectory("Back to Center", 0.4, 0.4, true,
            new Pose2d(1.100, 1.200, new Rotation2d(Math.toRadians(-90))), 
            List.of(
            ),
            new Pose2d(0.438, 5.080, new Rotation2d(Math.toRadians(-90)))
        );
        
        cache[TrajectoryType.centertoCenterBall.value] = calcTrajectory("Center to Center Ball", 0.3, 0.3, true,
            new Pose2d(5.080, 6.553, new Rotation2d(Math.toRadians(180))), 
            List.of(
            ),
            new Pose2d(6.548, 6.761, new Rotation2d(Math.toRadians(180)))
        );

        /* Four ball to back */

        cache[TrajectoryType.centerBallToBackFourball.value] = calcTrajectory("CenterBall to Back Fourball", 0.3, 0.3, false,
            new Pose2d(AutoConstants.ballCenterX, AutoConstants.ballCenterY, new Rotation2d(Math.toRadians(-135))), 
            List.of(
                            ),
            new Pose2d(AutoConstants.ballBackX, AutoConstants.ballBackY, new Rotation2d(Math.toRadians(168)))
        );

        cache[TrajectoryType.backToCenterFourBall.value] = calcTrajectory("Back to Center Fourball", 0.4, 0.4, false,
            new Pose2d(AutoConstants.ballBackX, AutoConstants.ballBackY, new Rotation2d(Math.toRadians(-135))), 
            List.of(
            ),
            new Pose2d(AutoConstants.ballCenterX, AutoConstants.ballCenterY, new Rotation2d(Math.toRadians(0)))
        );
        
        cache[TrajectoryType.leftToLeftBall.value] = calcTrajectory("Left to Left Ball", 0.3, 0.3, true,
            new Pose2d(5.000, 6.200, new Rotation2d(Math.toRadians(180))), 
            List.of(
            ),
            new Pose2d(6.016, 6.200, new Rotation2d(Math.toRadians(180)))
        );

    }


    /**
     * Builds a single trajectory based on the parameters passed in:
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param setReversed true = robot drives backwards, false = robot drives forwards
     * @param startPose Pose2d starting position (coordinates and angle)
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle)
     * @return trajectory that is generated
     */
    private Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        boolean setReversed, Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose) {
		Trajectory trajectory = null;
		DifferentialDriveKinematics driveKinematics = TrajectoryUtil.getDriveKinematics();
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"trackWidth",DriveConstants.TRACK_WIDTH,
				"maxVoltage", DriveConstants.MAX_VOLTAGE_IN_TRAJECTORY, 
				"kS", DriveConstants.kS, 
				"kV", DriveConstants.kV, 
				"kA", DriveConstants.kA,
				"maxSpeed", DriveConstants.kMaxSpeedMetersPerSecond,
				"maxAcceleration", DriveConstants.kMaxAccelerationMetersPerSecondSquared);

			// Create a voltage constraint to ensure we don't accelerate too fast
			DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA), 
				driveKinematics,
				DriveConstants.MAX_VOLTAGE_IN_TRAJECTORY);

			// Create config for trajectory
			TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond * maxVelRatio,
				DriveConstants.kMaxAccelerationMetersPerSecondSquared * maxAccelRatio)
				.setKinematics(driveKinematics)
				.addConstraint(autoVoltageConstraint)
				.setReversed(setReversed);			// Set to true if robot is running backwards

            // Generate the trajectory
			trajectory = TrajectoryGenerator.generateTrajectory(
				startPose, interriorWaypoints, endPose, config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, "SUCCESS", true);
		};
	
		return trajectory;
	}

}
