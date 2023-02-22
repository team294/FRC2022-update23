// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

/**
 * A class representing field coordinates.
 * <p> Field coordinates include:
 * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
 * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
 * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
 */
public class Field {
    //Robot probably 31" with bumpers

    //Community -> Loading
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] BlueCommunityColumnInitial = {
        new Pose2d(2.0871258795062873, 0.512826, new Rotation2d(Math.PI)), //54.25+(31/2)*sqrt(2)+6 inches to meters for x value
        new Pose2d(2.0871258795062873, 1.071626, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 1.630426, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 2.189226, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 2.748026, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 3.306826, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 3.865626, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 4.424426, new Rotation2d(Math.PI)), 
        new Pose2d(2.0871258795062873, 4.983226, new Rotation2d(Math.PI)) 
    };

    //Community -> Place part
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] BlueCommunityColumnFinal = {
        new Pose2d(1.77165, 0.512826, new Rotation2d(Math.PI)), //54.25+(31/2) inches to meters for x value
        new Pose2d(1.77165, 1.071626, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 1.630426, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 2.189226, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 2.748026, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 3.306826, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 3.865626, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 4.424426, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 4.983226, new Rotation2d(Math.PI)) 
    };

    //Loading -> Community
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] RedCommunityColumnInitial = {
        new Pose2d(2.0871258795062873, 3.020568, new Rotation2d(Math.PI)), //118.92 inches
        new Pose2d(2.0871258795062873, 3.579368, new Rotation2d(Math.PI)), //140.92 inches
        new Pose2d(2.0871258795062873, 4.138168, new Rotation2d(Math.PI)), //162.92 inches
        new Pose2d(2.0871258795062873, 4.696968, new Rotation2d(Math.PI)), //184.92 inches
        new Pose2d(2.0871258795062873, 5.255768, new Rotation2d(Math.PI)), //206.92 inches
        new Pose2d(2.0871258795062873, 5.814568, new Rotation2d(Math.PI)), //228.92 inches
        new Pose2d(2.0871258795062873, 6.373368, new Rotation2d(Math.PI)), //250.92 inches
        new Pose2d(2.0871258795062873, 6.932168, new Rotation2d(Math.PI)), //272.92 inches
        new Pose2d(2.0871258795062873, 7.490968, new Rotation2d(Math.PI))  //294.92 inches
    };

    //Community -> Place part
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] RedCommunityColumnFinal = {
        new Pose2d(1.77165, 3.020568, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 3.579368, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 4.138168, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 4.696968, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 5.255768, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 5.814568, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 6.373368, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 6.932168, new Rotation2d(Math.PI)), 
        new Pose2d(1.77165, 7.490968, new Rotation2d(Math.PI)) 
    };

    // #0 -> 2 = right to left, closest to driver (from driver point of view)
    // #3 -> 5 = right to left, furthest from driver (from driver point of view)
    //  5  4  3
    //  Station
    //  2  1  0
    // Community
    private final Pose2d[] BlueStationInitial = {
        new Pose2d(2.148713, 2.130489, new Rotation2d(0)),
        new Pose2d(2.148713, 2.748026, new Rotation2d(0)),
        new Pose2d(2.148713, 3.365563, new Rotation2d(0)),
        new Pose2d(4.855718, 2.130489, new Rotation2d(Math.PI)),
        new Pose2d(4.855718, 2.748026, new Rotation2d(Math.PI)),
        new Pose2d(4.855718, 3.365563, new Rotation2d(Math.PI))
    };
    
    //Community/Field -> Station
    // #0 = furthest to right (from driver point of view)
    // #2 = furthest to left (from driver point of view)
    private final Pose2d[] BlueStationFinal = {
        new Pose2d(3.8354, 2.130489, new Rotation2d(0)), //Always faces away from communities, this could cause issues
        new Pose2d(3.8354, 2.748026, new Rotation2d(0)),
        new Pose2d(3.8354, 3.365563, new Rotation2d(0))
    };

    // #0 -> 2 = right to left, closest to driver (from driver point of view)
    // #3 -> 5 = right to left, furthest from driver (from driver point of view)
    //  5  4  3
    //  Station
    //  2  1  0
    // Community
    private final Pose2d[] RedStationInitial = {
        new Pose2d(2.148713, 4.646867, new Rotation2d(0)),
        new Pose2d(2.148713, 5.264404, new Rotation2d(0)),
        new Pose2d(2.148713, 5.881941, new Rotation2d(0)),
        new Pose2d(4.855718, 4.646867, new Rotation2d(Math.PI)),
        new Pose2d(4.855718, 5.264404, new Rotation2d(Math.PI)),
        new Pose2d(4.855718, 5.881941, new Rotation2d(Math.PI))
    };
    
    //Community/Field -> Station
    // #0 = furthest to right (from driver point of view)
    // #2 = furthest to left (from driver point of view)
    private final Pose2d[] RedStationFinal = {
        new Pose2d(3.8354, 4.646867, new Rotation2d(0)), //Always faces away from communities, this could cause issues
        new Pose2d(3.8354, 5.264404, new Rotation2d(0)), //Values found by adding loading zone width (99.07 inches) to Blue values
        new Pose2d(3.8354, 5.881941, new Rotation2d(0))
    };

    // #0 -> 3 = red(far) side, right to left (from driver point of view)
    // #4 -> 7 = blue(close) side, left to right (from driver point of view)
    // #3 and 4 are loading zone tags
    private final AprilTag[] AprilTagsBlue = {
        new AprilTag(1, new Pose3d(new Pose2d(15.51356, 1.071626, new Rotation2d(0)))),
        new AprilTag(2, new Pose3d(new Pose2d(15.51356, 2.748026, new Rotation2d(0)))),
        new AprilTag(3, new Pose3d(new Pose2d(15.51356, 4.424426, new Rotation2d(0)))),
        new AprilTag(4, new Pose3d(new Pose2d(16.17878, 6.749796, new Rotation2d(0)))),
        new AprilTag(5, new Pose3d(new Pose2d(0.36195, 6.749796, new Rotation2d(Math.PI)))),
        new AprilTag(6, new Pose3d(new Pose2d(1.02743, 4.424426, new Rotation2d(Math.PI)))),
        new AprilTag(7, new Pose3d(new Pose2d(1.02743, 2.748026, new Rotation2d(Math.PI)))),
        new AprilTag(8, new Pose3d(new Pose2d(1.02743, 1.071626, new Rotation2d(Math.PI))))
    };
    
    // #0 -> 3 = red(close) side, left to right (from driver point of view)
    // #4 -> 7 = blue(far) side, right to left (from driver point of view)
    // #3 and 4 are loading zone tags
    private final AprilTag[] AprilTagsRed = {
        new AprilTag(1, new Pose3d(new Pose2d(1.02743, 6.932168, new Rotation2d(Math.PI)))),
        new AprilTag(2, new Pose3d(new Pose2d(1.02743, 5.255768, new Rotation2d(Math.PI)))),
        new AprilTag(3, new Pose3d(new Pose2d(1.02743, 3.579368, new Rotation2d(Math.PI)))),
        new AprilTag(4, new Pose3d(new Pose2d(0.36195, 1.253998, new Rotation2d(Math.PI)))),
        new AprilTag(5, new Pose3d(new Pose2d(16.17878, 1.253998, new Rotation2d(0)))),
        new AprilTag(6, new Pose3d(new Pose2d(15.51356, 3.579368, new Rotation2d(0)))),
        new AprilTag(7, new Pose3d(new Pose2d(15.51356, 5.255768, new Rotation2d(0)))),
        new AprilTag(8, new Pose3d(new Pose2d(15.51356, 6.932168, new Rotation2d(0))))
    };

    private final AllianceSelection alliance;
    private final FileLog log;

    /**
     * Create a field object that can provide various field locations.  All field
     * locations are Pose2d objects based on the current alliance that is selected.
     * Pose components include:
     * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
     * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
     * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
     * @param alliance Alliance object to provide the currently selected alliance
     */
    public Field(AllianceSelection alliance, FileLog log){
        this.alliance = alliance;
        this.log = log;
    }

    /**
	 * Gets the initial column position (in front of scoring position, but backed up with a little room for the robot to rotate).
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
     * <p> #1 = furthest to right (from driver point of view)
     * <p> #9 = furthest to left (from driver point of view)
	 * 
	 * @param column The column that will be returned (1-9)
	 */
    public Pose2d getInitialColumn(int column) throws IndexOutOfBoundsException {
        if(column < 10 && column > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueCommunityColumnInitial[column-1];
            }
            else {
                return RedCommunityColumnInitial[column-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Initial Column %d out of range", column));
        }
    }

    /**
	 * Gets the column scoring position.
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
     * <p> #1 = furthest to right (from driver point of view)
     * <p> #9 = furthest to left (from driver point of view)
	 * 
	 * @param column The column that will be returned (1-9)
	 */
    public Pose2d getFinalColumn(int column) throws IndexOutOfBoundsException {
        if(column < 10 && column > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueCommunityColumnFinal[column-1];
            }
            else {
                return RedCommunityColumnFinal[column-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Column ID %d out of range", column));
        }
    }

    /**
	 * Gets the position to approach the station from
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position 1-3 Lowest-Highest Communtiy Side | 4-6 Lowest-Highest Field Side
	 */
    public Pose2d getStationInitial(int position) throws IndexOutOfBoundsException {
        if(position < 7 && position > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueStationInitial[position-1];
            }
            else {
                return RedStationInitial[position-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Station Position %d out of range", position));
        }
    }

    /**
	 * Gets the center positions on the station
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position 1-3 Lowest-Hightest (Y-Axis)
	 */
    public Pose2d getStationCenter(int position) throws IndexOutOfBoundsException {
        if(position < 4 && position > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueStationFinal[position-1];
            }
            else {
                return RedStationFinal[position-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Station Center position %d out of range", position));
        }
    }

    /**
	 * Gets the position of a specified April Tag (1-8)
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position
	 */
    public AprilTag getAprilTag(int ID) throws IndexOutOfBoundsException {
        if(ID < 9 && ID > 0) {
            if(alliance.getAlliance() == Alliance.Blue) {
                return AprilTagsBlue[ID-1];
            } else {
                return AprilTagsRed[ID-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("AprilTag ID %d out of range", ID));
        }
    }

    public AprilTagFieldLayout getAprilTagFieldLayout() {
        List<AprilTag> atList;

        
        if(alliance.getAlliance() == Alliance.Blue) {
            atList = Arrays.asList(AprilTagsBlue);
        } else {
            atList = Arrays.asList(AprilTagsRed);
        }

        

        return new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);
    }

    /**
     * Returns the column of the closest goal (1-9), based on the current game piece setting in the manipulator.
     * <p> #1 = furthest to right (from driver point of view)
     * <p> #9 = furthest to left (from driver point of view)
     * @return Column of the closest goal (1-9) for the current game piece setting in the manipulator
     */
    public int getClosestGoal(Pose2d robotPose, boolean isCone) {
        int closestGoal;
        double robotY = robotPose.getY();

        if(alliance.getAlliance() == Alliance.Blue){//Alliance Blue
            closestGoal = 0;
            if(isCone){//Carrying Cone
                for(int i = 0; i < 9; i++){
                    if(i == 1 || i == 4 || i == 7){
                        continue;
                    }
                    if(Math.abs(robotY - BlueCommunityColumnFinal[i].getY()) < Math.abs(robotY - BlueCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            } else {
                closestGoal = 1;
                for(int i = 1; i < 9; i++){
                    if(i != 1 && i != 4 && i != 7){
                        continue;
                    }
                    if(Math.abs(robotY - BlueCommunityColumnFinal[i].getY()) < Math.abs(robotY - BlueCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            }
            log.writeLogEcho(true, "Field", "GetClosetGoal", "Alliance", "Blue", "Cone", isCone, 
                "Column", closestGoal+1, "X", BlueCommunityColumnFinal[closestGoal].getX(),
                "Y", BlueCommunityColumnFinal[closestGoal].getY(), "Rot", BlueCommunityColumnFinal[closestGoal].getRotation().getDegrees());
        } else {
            closestGoal = 0;
            if(isCone){//Carrying Cone
                for(int i = 0; i < 9; i++){
                    if(i == 1 || i == 4 || i == 7){
                        continue;
                    }
                    if(Math.abs(robotY - RedCommunityColumnFinal[i].getY()) < Math.abs(robotY - RedCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            } else {
                closestGoal = 1;
                for(int i = 1; i < 9; i++){
                    if(i != 1 && i != 4 && i != 7){
                        continue;
                    }
                    if(Math.abs(robotY - RedCommunityColumnFinal[i].getY()) < Math.abs(robotY - RedCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            }
            log.writeLogEcho(true, "Field", "GetClosetGoal", "Alliance", "Red", "Cone", isCone, 
                "Column", closestGoal+1, "X", RedCommunityColumnFinal[closestGoal].getX(),
                "Y", RedCommunityColumnFinal[closestGoal].getY(), "Rot", RedCommunityColumnFinal[closestGoal].getRotation().getDegrees());
        }
        closestGoal++;          // Adjust for 0-based index in array
        SmartDashboard.putNumber("Closest Goal", closestGoal);
        return closestGoal;
    }
}
