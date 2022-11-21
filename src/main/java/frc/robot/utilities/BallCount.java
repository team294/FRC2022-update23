// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BallLocation;

/** Add your docs here. */
public class BallCount {
    private static int[] count = new int[]{0, 0, 0};
    /**
     * 
     * @param ballLocation location where the ball is added to
     */
    public static void addBall(BallLocation ballLocation, FileLog log){
        count[ballLocation.value]++;
        log.writeLog(false, "Ball Count", "Ball Added", "Location", ballLocation.name());
        updateShuffleboard();
    }

    /**
     * 
     * @param ballLocation location where the ball is subtracted from
     */
    public static void subtractBall(BallLocation ballLocation, FileLog log){
       count[ballLocation.value]--;
       log.writeLog(false, "Ball Count", "Ball Subtracted", "Location", ballLocation.name());
       updateShuffleboard();
    }

    public static void setBallCount(int numBalls, BallLocation ballLocation, FileLog log){
        count[ballLocation.value] = numBalls;
    }

    /**
     * 
     * @return Total Balls in the robot
     */
    public static int getTotalBallCount(){
        return count[0] + count[1] + count[2];
    }

    /**
     * 
     * @param ballLocation location where the balls are
     * @return number of balls in ballLocation
     */
    public static int getBallCount(BallLocation ballLocation){
        return count[ballLocation.value];
    }

    public static void updateShuffleboard(){
        SmartDashboard.putNumber("Balls in Eject", count[BallLocation.kEject.value]);
        SmartDashboard.putNumber("Balls in Uptake", count[BallLocation.kUptake.value]);     
        SmartDashboard.putNumber("Balls in Feeder", count[BallLocation.kFeeder.value]);
    }

   
}
