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
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

import static frc.robot.Constants.DriveConstants.*;

public class DriveTurnGyro extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private DriveTrain driveTrain; // reference to driveTrain
  private double target; // how many more degrees to the right to turn
  private double direction; // +1 = turn to the left, -1 = turn to the right
  private double maxVel; // max velocity, between 0 and kMaxAngularVelocity in Constants
  private double maxAccel; // max acceleration, between 0 and kMaxAngularAcceleration in Constants
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startAngle, targetRel; // starting angle in degrees, target angle relative to start angle
  private double currAngle, currVelocity, currVelocityGyro;
  private double timeSinceStart;
  private TargetType targetType;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private FileLog log;
  private LimeLight limeLight;
  private PIDController pidAngVel;
  private double angleTolerance;
  private boolean feedbackUsingVision, nearTargetAngle;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNow; // next state of the system (next loop cycle) as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateForecast; // state of the system in the future, as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * Turns the robot to a target angle.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use limelight to turn towards the goal)
   * @param maxVel max velocity in degrees/sec, between 0 and kMaxAngularVelocity in Constants
   * @param maxAccel max acceleration in degrees/sec2, between 0 and kMaxAngularAcceleration in Constants
   * @param regenerate true to regenerate profile while running
   * @param angleTolerance the tolerance to use for turn gyro
   * @param driveTrain drivetrain
   * @param limeLight limelight
   * @param log log
   */
  public DriveTurnGyro(TargetType type, double target, double maxVel, double maxAccel, boolean regenerate, double angleTolerance, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.target = driveTrain.normalizeAngle(target);
    this.targetType = type;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, DriveConstants.kMaxAngularVelocity);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, DriveConstants.kMaxAngularAcceleration);
    this.regenerate = regenerate;
    this.fromShuffleboard = false;
    this.angleTolerance = Math.abs(angleTolerance);

    addRequirements(driveTrain, limeLight);

    pidAngVel = new PIDController(kPAngular, 0, kDAngular);
  }

  /**
   * To be used when changing the target value directly from shuffleboard (not a pre-coded target)
   * @param fromShuffleboard true means the value is being changed from shuffleboard
   */
  public DriveTurnGyro(TargetType type, boolean regenerate, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    this.log = log;
    this.target = 0;
    this.targetType = type;
    this.maxVel = 0;
    this.maxAccel = 0;
    this.regenerate = regenerate;
    this.fromShuffleboard = true;
    this.angleTolerance = 0;
    addRequirements(driveTrain);

    if(SmartDashboard.getNumber("TurnGyro Manual Target Ang", -9999) == -9999) {
      SmartDashboard.putNumber("TurnGyro Manual Target Ang", 90);
    }
    if(SmartDashboard.getNumber("TurnGyro Manual MaxVel", -9999) == -9999) {
      SmartDashboard.putNumber("TurnGyro Manual MaxVel", 90);
    }
    if(SmartDashboard.getNumber("TurnGyro Manual MaxAccel", -9999) == -9999) {
      SmartDashboard.putNumber("TurnGyro Manual MaxAccel", 100);
    }
    if(SmartDashboard.getNumber("TurnGyro Manual Tolerance", -9999) == -9999) {
      SmartDashboard.putNumber("TurnGyro Manual Tolerance", 2);
    }

    pidAngVel = new PIDController(kPAngular, 0, kDAngular);
  }
 
  /**
   * Turns the robot to a target angle.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use limelight to turn towards the goal)
   * @param target degrees to turn from +180 (left) to -180 (right) [ignored for kVisionOnScreen]
   * @param maxVel max velocity in degrees/sec, between 0 and kMaxAngularVelocity in Constants
   * @param maxAccel max acceleration in degrees/sec2, between 0 and kMaxAngularAcceleration in Constants
   * @param angleTolerance the tolerance to use for turn gyro
   * @param driveTrain drivetrain
   * @param limeLight limelight
   * @param log log
   */
  public DriveTurnGyro(TargetType type, double target, double maxVel, double maxAccel, double angleTolerance, DriveTrain driveTrain, LimeLight limeLight, FileLog log) {
    this(type, target, maxVel, maxAccel, true, angleTolerance, driveTrain, limeLight, log);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nearTargetAngle = false;
    feedbackUsingVision = false;
    driveTrain.setDriveModeCoast(false);
    driveTrain.setOpenLoopRampLimit(false);

    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("TurnGyro Manual Target Ang", 90);
      maxVel = SmartDashboard.getNumber("TurnGyro Manual MaxVel", kMaxAngularVelocity*0.08);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, DriveConstants.kMaxAngularVelocity);
      maxAccel = SmartDashboard.getNumber("TurnGyro Manual MaxAccel", kMaxAngularAcceleration);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, DriveConstants.kMaxAngularAcceleration);
      angleTolerance = SmartDashboard.getNumber("TurnGyro Manual Tolerance", 2);
      angleTolerance = Math.abs(angleTolerance);
    }
    // If constants were updated from Shuffleboard, then update PID
    pidAngVel.setPID(kPAngular, 0, kDAngular);
    pidAngVel.reset();


    startAngle = driveTrain.getGyroRotation();

    switch (targetType) {
      case kRelative:
        targetRel = target;
        break;
      case kAbsolute:
        targetRel = driveTrain.normalizeAngle(target - startAngle);
        break;
      case kVisionOnScreen: case kVisionScanLeft: case kVisionScanRight:
        targetRel = driveTrain.normalizeAngle(limeLight.getXOffset());
        limeLight.enableFastLogging(true);
        break;
    }

    direction = Math.signum(targetRel);

    tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, driveTrain.getAngularVelocityFromWheels()); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    // initialize velocity and accel limits
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel , maxAccel);
    // generate profile
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;

    log.writeLog(false, "DriveTurnGyro", "initialize", "Total Time", tProfile.totalTime(), "StartAngleAbs", startAngle, "TargetAngleRel", targetRel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pFF, pFB, pDB;  // variables for feed forward power, feedback power, and deadband power

    currProfileTime = System.currentTimeMillis();
    // currAngle is relative to the startAngle.  -90 to +270 if turning left, +90 to -270 if turning right.
    currAngle = driveTrain.normalizeAngle(driveTrain.getGyroRotation() - startAngle);
    currAngle += (direction*currAngle<-90) ? direction*360.0 : 0; 
    currVelocity = driveTrain.getAngularVelocityFromWheels();
    currVelocityGyro = driveTrain.getAngularVelocity();
    
    if (targetType == TargetType.kVisionOnScreen) {
      targetRel = driveTrain.normalizeAngle(currAngle + limeLight.getXOffset());
      tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0);
      if(limeLight.canTakeSnapshot()) {
        limeLight.setSnapshot(true);
      }    
    }
    timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    tStateNow = tProfile.calculate(timeSinceStart);        // This is where the robot should be now
    tStateForecast = tProfile.calculate(timeSinceStart + tLagAngular);  // This is where the robot should be next cycle (or farther in the future if the robot has lag or backlash)

    if(tProfile.isFinished(timeSinceStart) && targetType == TargetType.kVisionOnScreen){
      // If we completed the trapezoid profile and we are using vision, then
      // fine-tune angle using feedback based on live camera feedback
      feedbackUsingVision = true;
    }

    targetVel = tStateNow.velocity;
    targetAccel = tStateNow.acceleration;
    double forecastVel = tStateForecast.velocity;
    double forecastAccel = MathUtil.clamp((forecastVel-targetVel)/tLagAngular, -maxAccel, maxAccel);

    if (Math.abs(targetRel - currAngle) < 5) {
      // We are within a few degrees (5 degrees was initial value above)
      nearTargetAngle = true;
    }

    if(!tProfile.isFinished(timeSinceStart) && !nearTargetAngle){
      // Feed-forward percent voltage to drive motors
      pFF = (forecastVel * kVAngular) + (forecastAccel * kAAngular);
      // Normal feedback for following trapezoid profile
      pFB = MathUtil.clamp(pidAngVel.calculate(currVelocity, targetVel) + kIAngular * (tStateNow.position - currAngle), -0.1, 0.1);
    } else {
      // We are past the end of the trapezoid profile
      forecastVel = 0.0;
      forecastAccel = 0.0;
      pFF = 0.0;

      if (targetType == TargetType.kVisionOnScreen) {
        // Live camera feedback
        // TODO make the fixed pFB speed below (0.01) a constant.  Increase value slowly if the robot is not moving.
        pFB = 0.01 * Math.signum( driveTrain.normalizeAngle(limeLight.getXOffset()) );
      } else {
        // Bang-bang control
        // TODO make the fixed pFB speed below (0.01) a constant.  Increase value slowly if the robot is not moving.
        pFB = 0.01 * Math.signum( targetRel - currAngle );
      }
    }
    
    // Use kS to account for dead band
    pDB = kSAngular * Math.signum(pFF + pFB);

    driveTrain.setLeftMotorOutput(-pFF - pFB - pDB);
    driveTrain.setRightMotorOutput(+pFF + pFB + pDB);

    if (regenerate) {
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, currVelocity);
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, targetVel);  
      // tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

      // When regenerating using tStateNow, the profile only changes if the final state has moved (such as due to vision)
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateNow);
      profileStartTime = currProfileTime;
    }

    log.writeLog(false, "DriveTurnGyro", "profile", "target", targetRel, 
      "posT", tStateNow.position, "velT", targetVel, "accT", targetAccel,
      "posF", tStateForecast.position, "velF", forecastVel, "accF", forecastAccel,
      "posA", currAngle, "velAWheel", currVelocity, "velAGyro", currVelocityGyro, "pFF", pFF, "pFB", pFB, "pTotal", pFF+pFB+pDB, "LL x", limeLight.getXOffset(), "LL y", limeLight.getYOffset());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotorOutput(0);
    driveTrain.setRightMotorOutput(0);
    driveTrain.setDriveModeCoast(false);
    driveTrain.setOpenLoopRampLimit(true);
    
    if (targetType == TargetType.kVisionOnScreen) {
      limeLight.enableFastLogging(false);
    }
    
    log.writeLog(false, "DriveTurnGyro", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if ((targetType != TargetType.kVisionOnScreen && Math.abs(targetRel - currAngle) < angleTolerance) || 
        (targetType == TargetType.kVisionOnScreen && limeLight.seesTarget() && Math.abs(limeLight.getXOffset()) < angleTolerance) ||
        (targetType == TargetType.kVisionOnScreen && !limeLight.seesTarget() && feedbackUsingVision) ) {
      accuracyCounter++;
      log.writeLog(false, "DriveTurnGyro", "WithinTolerance", "Target Ang", targetRel, "Actual Ang", currAngle, 
        "LimeLight Xoff", limeLight.getXOffset(), "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    if(accuracyCounter >= 5) {
      return true;
    }
    return false;
  }
}