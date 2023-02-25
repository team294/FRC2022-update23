/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import static frc.robot.Constants.DriveConstants.*;

public class Balance extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private LimeLight limeLight;
  private TargetType angleType;
  private double target; // how many more degrees to the right to turn
  private double direction;  // +1 = forward, -1 = reverse
  private double maxVel; // max velocity, between 0 and kMaxSpeedMetersPerSecond in Constants 
  private double maxAccel; // max acceleration, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
  private long profileStartTime; // initial time (time of starting point)
  private double startDistLeft, startDistRight;
  private double currDist;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private double angleInput, angleTarget;   // angleTarget is an absolute gyro angle
  private FileLog log;

  private boolean up = false;
  private boolean down = false;
  private float max = 0;
  private float roll = 0;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNext; // next state of the system as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system
 
  /**
   * Drives the robot straight.
   * @param target distance to travel, in meters
   * @param angleType kRelative (angle is relative to current robot facing),
   *   kAbsolute (angle is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use limelight to drive towards the goal)
   * @param angle angle to drive along when driving straight (+ = left, - = right)
   * @param maxVel max velocity in meters/second, between 0 and kMaxSpeedMetersPerSecond in Constants
   * @param maxAccel max acceleration in meters/second2, between 0 and kMaxAccelerationMetersPerSecondSquared in Constants
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param driveTrain reference to the drive train subsystem
   * @param limelight reference to the LimeLight subsystem
   * @param log
   */
  public Balance(double target, TargetType angleType, double angle, double maxVel, double maxAccel, boolean regenerate, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.angleType = angleType;
    angleInput = angle;
    this.regenerate = regenerate;
    this.fromShuffleboard = false;
    this.target = target;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, DriveConstants.kMaxSpeedMetersPerSecond);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, DriveConstants.kMaxAccelerationMetersPerSecondSquared);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }


  /**
   * Use this constructor when reading values from Shuffleboard
   * @param angleType kRelative (angle is relative to current robot facing),
   *   kAbsolute (angle is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use limelight to drive towards the goal)
   * @param regenerate true = regenerate profile each cycle (to accurately reach target distance), false = don't regenerate (for debugging)
   * @param driveTrain reference to the drive train subsystem
   * @param limelight reference to the LimeLight subsystem
   * @param log
   */
  public Balance(TargetType angleType, boolean regenerate, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.angleType = angleType;
    angleInput = 0;
    this.regenerate = regenerate;
    this.fromShuffleboard = true;
    this.target = 0;
    this.maxVel = 0.5 * DriveConstants.kMaxSpeedMetersPerSecond;
    this.maxAccel = 0.5 * DriveConstants.kMaxAccelerationMetersPerSecondSquared;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    if(SmartDashboard.getNumber("DriveStraight Manual Target Dist", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Target Dist", 2);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual Angle", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual Angle", 0);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual MaxVel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxVel", kMaxSpeedMetersPerSecond);
    }
    if(SmartDashboard.getNumber("DriveStraight Manual MaxAccel", -9999) == -9999) {
      SmartDashboard.putNumber("DriveStraight Manual MaxAccel", kMaxAccelerationMetersPerSecondSquared);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("DriveStraight Manual Target Dist", 2);
      angleInput = SmartDashboard.getNumber("DriveStraight Manual Angle", 0);
      maxVel = SmartDashboard.getNumber("DriveStraight Manual MaxVel", kMaxSpeedMetersPerSecond);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, DriveConstants.kMaxSpeedMetersPerSecond);
      maxAccel = SmartDashboard.getNumber("DriveStraight Manual MaxAccel", kMaxAccelerationMetersPerSecondSquared);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, DriveConstants.kMaxAccelerationMetersPerSecondSquared);
    }

    // Set angleTarget to absolute gyro target angle
    switch (angleType) {
      case kRelative:
        angleTarget = driveTrain.normalizeAngle(driveTrain.getGyroRotation() + angleInput);
        break;
      case kAbsolute:
        angleTarget = driveTrain.normalizeAngle(angleInput);
        break;
      case kVisionOnScreen: case kVisionScanLeft: case kVisionScanRight:
        angleTarget = driveTrain.normalizeAngle(driveTrain.getGyroRotation() + limeLight.getXOffset());
    }

    // if (sweetSpot && limeLight.seesTarget()) {
    //     target = Units.inchesToMeters(limeLight.getSweetSpot() * 12);
    // }

    direction = Math.signum(target);

    tStateFinal = new TrapezoidProfileBCR.State(target, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, 0.0); // initialize initial state (relative turning, so assume initPos is 0 degrees)
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel, maxAccel); // initialize velocity and accel limits
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr); // generate profile
    log.writeLog(false, "DriveStraight", "init", "Target", target, "Profile total time", tProfile.totalTime());
    
    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    startDistLeft = Units.inchesToMeters(driveTrain.getLeftEncoderInches());
    startDistRight = Units.inchesToMeters(driveTrain.getRightEncoderInches());
    
    driveTrain.setTalonPIDConstants(kPLinear, kILinear, kDLinear, 0);
    driveTrain.resetTalonPIDs();
    driveTrain.setDriveModeCoast(false);

    up = false;
    down = false;
    roll = 0;
    max = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update data for this iteration
    long currProfileTime = System.currentTimeMillis();
    double timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    double currDistLeft = Units.inchesToMeters(driveTrain.getLeftEncoderInches()) - startDistLeft;
    double currDistRight = Units.inchesToMeters(driveTrain.getRightEncoderInches()) - startDistRight;
    currDist = (currDistLeft + currDistRight) * 0.5;

    // Get next state from trapezoid profile
    tStateNext = tProfile.calculate(timeSinceStart + 0.010);
    double targetVel = tStateNext.velocity;
    double targetAccel = tStateNext.acceleration;

    // Calculate correction to maintain angle
    double curAngle = driveTrain.getGyroRotation();
    if (angleType == TargetType.kVisionOnScreen) {
      angleTarget = driveTrain.normalizeAngle(curAngle + limeLight.getXOffset());
      if(limeLight.canTakeSnapshot()) {
        limeLight.setSnapshot(true);
      }    
    }
    double pAngle = driveTrain.normalizeAngle(curAngle - angleTarget) * kAngLinear;
    double targetVelL = targetVel * (1 + direction*pAngle);
    double targetVelR = targetVel * (1 - direction*pAngle);
    
    // Calculate feedforward power
    double aFFL = (kSLinear * Math.signum(targetVelL)) + (targetVelL * kVLinear) + (targetAccel * kALinear);
    double aFFR = (kSLinear * Math.signum(targetVelR)) + (targetVelR * kVLinear) + (targetAccel * kALinear);

    // look at pitch to determine if we have balanced
    roll = driveTrain.getGyroRoll();

    if (roll > max) {
      max = roll;
    }

    if (!up && roll > 12) {
      up = true;
      log.writeLog(false, "Balance", "Execute", "Up",roll);
    }

    if (up && !down && roll < 12) {
      down = true;
      log.writeLog(false, "Balance", "Execute", "Down",roll);
    }

   // For calibrating:  driving with feedforward only
    // driveTrain.setOpenLoopRampLimit(false);
    // driveTrain.setLeftMotorOutput(aFFL);
    // driveTrain.setRightMotorOutput(aFFR);

    // For competition:  driving with feedforward and feedback
    driveTrain.setLeftTalonPIDVelocity(Units.metersToInches(targetVelL), aFFL);
    driveTrain.setRightTalonPIDVelocity(Units.metersToInches(targetVelR), aFFR, true);


    log.writeLog(false, "Balance", "profile", 
      "roll", roll,
      "angT", angleTarget, "angA", curAngle,
      "posT", tStateNext.position, 
      "velLT", targetVelL, "velRT", targetVelR, "accT", targetAccel,
      "posA", currDist, "posLA", currDistLeft, "posRA", currDistRight, 
      "velLA", Units.inchesToMeters(driveTrain.getLeftEncoderVelocity()), 
      "velRA", Units.inchesToMeters(driveTrain.getRightEncoderVelocity()), 
      "aFFL", aFFL, "aFFR", aFFR,
      "pctOutLA", driveTrain.getLeftOutputPercent(), "VoutNormA", driveTrain.getLeftOutputVoltage()/compensationVoltage, "VbusLA", driveTrain.getLeftBusVoltage(),
      "velRawLA", driveTrain.getLeftEncoderVelocityRaw(), "errRawLA", driveTrain.getTalonLeftClosedLoopError(), 
      "targetRawL", driveTrain.getTalonLeftClosedLoopTarget());

    double linearVel = Units.inchesToMeters(driveTrain.getAverageEncoderVelocity());
    if(regenerate) {
      tStateCurr = new TrapezoidProfileBCR.State(currDist, linearVel);
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);
      profileStartTime = currProfileTime;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "Balance", "End","Interrupted",interrupted);
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
    driveTrain.setDriveModeCoast(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (up && roll < (max - 3.0)) {
    //if (up && down) {
      log.writeLog(false, "Balance", "isFinished", "Roll", roll, "Max", max, "Up",up, "Down", down);
      return true;
    }    

    if(Math.abs(target - currDist) < 0.0125) {
      accuracyCounter++;
      log.writeLog(false, "DriveStraight", "WithinTolerance", "Target Dist", target, "Actual Dist", currDist, "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    return (accuracyCounter >= 5);
  }
}