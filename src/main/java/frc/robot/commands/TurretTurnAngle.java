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
import frc.robot.Constants.TargetType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.MathBCR;

import static frc.robot.Constants.TurretConstants.*;

public class TurretTurnAngle extends CommandBase {
  /**
   * Uses wpilib TrapezoidProfile generator to generate a motion profile for drive train turning
   * Does not regenerate the profile every time
   */

  private Turret turret; // reference to turret subsystem
  private FileLog log;
  private double target; // how many more degrees to the right to turn
  private double maxVel; // max velocity, between 0 and kMaxAngularVelocity in Constants
  private double maxAccel; // max acceleration, between 0 and kMaxAngularAcceleration in Constants
  private long profileStartTime; // initial time (time of starting point)
  private long currProfileTime;
  private double targetVel; // velocity to reach by the end of the profile in deg/sec (probably 0 deg/sec)
  private double targetAccel;
  private double startAngle, targetRel; // starting angle in degrees, target angle relative to start angle
  private double currAngle, currVelocity;
  private double timeSinceStart;
  private TargetType targetType;
  private TargetType originalTargetType;
  private boolean regenerate;
  private boolean fromShuffleboard;
  private boolean encoderCalibrated = true;
  private boolean continualTracking;

  private PiVisionHub piVisionHub;
  private PIDController pidAngVel;
  private double angleTolerance;
  private boolean feedbackUsingVision;

  private int accuracyCounter = 0;

  private TrapezoidProfileBCR tProfile; // wpilib trapezoid profile generator
  private TrapezoidProfileBCR.State tStateCurr; // initial state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.State tStateNow; // next state of the system (next loop cycle) as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateForecast; // state of the system in the future, as calculated by the profile generator
  private TrapezoidProfileBCR.State tStateFinal; // goal state of the system (position in deg and time in sec)
  private TrapezoidProfileBCR.Constraints tConstraints; // max vel (deg/sec) and max accel (deg/sec/sec) of the system

  /**
   * Turns the robot to a target angle.
   * <p> This command DOES NOTHING if the turret is not calibrated.
   * <p> This is the EASIEST constructor to use for the TurrentTurnAngle command.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use camera to turn towards the goal),
   *   kVisionScanLeft (turn left until camera sees a target, then use camera to turn towards the goal),
   *   kVisionScanRight (turn right until camera sees a target, then use camera to turn towards the goal)
   * @param target degrees to turn from +180 (left) to -180 (right) [ignored for kVisionOnScreen]
   * @param angleTolerance the tolerance to use for turn gyro, negative angle = continuous tracking
   * @param turret turret subsystem
   * @param piVisionHub pivisionhub subsystem
   * @param log log
   */
  public TurretTurnAngle(TargetType type, double target, double angleTolerance, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    this(type, target, kClampTurnVelocity, kMaxTurnAcceleration, true, angleTolerance, turret, piVisionHub, log);
    // this(type, target, kClampTurnVelocity, kMaxTurnAcceleration, true, angleTolerance, turret, log);
  }

  /**
   * Turns the robot to a target angle.
   * <p> This command DOES NOTHING if the turret is not calibrated.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use camera to turn towards the goal),
   *   kVisionScanLeft (turn left until camera sees a target, then use camera to turn towards the goal),
   *   kVisionScanRight (turn right until camera sees a target, then use camera to turn towards the goal)
   * @param target degrees to turn from +180 (left) to -180 (right) [ignored for kVisionOnScreen]
   * @param maxVel max velocity in degrees/sec, between 0 and kMaxAngularVelocity in Constants
   * @param maxAccel max acceleration in degrees/sec2, between 0 and kMaxAngularAcceleration in Constants
   * @param angleTolerance the tolerance to use for turn gyro, negative angle = continuous tracking
   * @param turret turret subsystem
   * @param piVisionHub pivisionhub subsystem
   * @param log log
   */
  public TurretTurnAngle(TargetType type, double target, double maxVel, double maxAccel, double angleTolerance, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    this(type, target, maxVel, maxAccel, true, angleTolerance, turret, piVisionHub, log);
    // this(type, target, maxVel, maxAccel, true, angleTolerance, turret, log);
  }

  /**
   * Turns the robot to a target angle.
   * <p> This command DOES NOTHING if the turret is not calibrated.
   * @param type kRelative (target is an angle relative to current robot facing),
   *   kAbsolute (target is an absolute field angle; 0 = away from drive station),
   *   kVisionOnScreen (use camera to turn towards the goal),
   *   kVisionScanLeft (turn left until camera sees a target, then use camera to turn towards the goal),
   *   kVisionScanRight (turn right until camera sees a target, then use camera to turn towards the goal)
   * @param maxVel max velocity in degrees/sec, between 0 and kMaxAngularVelocity in Constants
   * @param maxAccel max acceleration in degrees/sec2, between 0 and kMaxAngularAcceleration in Constants
   * @param regenerate true to regenerate profile while running
   * @param angleTolerance the tolerance to use for turn gyro, negative angle = continuous tracking
   * @param turret turret subsystem
   * @param piVisionHub pivisionhub subsystem
   * @param log log
   */
  public TurretTurnAngle(TargetType type, double target, double maxVel, double maxAccel, boolean regenerate, double angleTolerance, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.piVisionHub = piVisionHub;
    this.log = log;
    this.target = MathBCR.normalizeAngle(target);
    this.originalTargetType = type;
    this.targetType = type;
    this.maxVel = MathUtil.clamp(Math.abs(maxVel), 0, kClampTurnVelocity);
    this.maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, kMaxTurnAcceleration);
    this.regenerate = regenerate;
    this.fromShuffleboard = false;
    continualTracking = angleTolerance < 0;
    this.angleTolerance = Math.abs(angleTolerance);


    addRequirements(turret);

    pidAngVel = new PIDController(kPTurn, 0, kDTurn);
  }

  /**
   * To be used when changing the target value directly from shuffleboard (not a pre-coded target)
   * @param fromShuffleboard true means the value is being changed from shuffleboard
   */
  public TurretTurnAngle(TargetType type, boolean regenerate, Turret turret, PiVisionHub piVisionHub, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.piVisionHub = piVisionHub;
    this.log = log;
    this.target = 0;
    this.originalTargetType = type;
    this.targetType = type;
    this.maxVel = 0;
    this.maxAccel = 0;
    this.regenerate = regenerate;
    this.fromShuffleboard = true;
    this.angleTolerance = 0;
    addRequirements(turret);

    SmartDashboard.putNumber("TurnTurret Manual Target Ang", 45);
    SmartDashboard.putNumber("TurnTurret Manual MaxVel", 300);  //150 during season, increased to 300 after tuning
    SmartDashboard.putNumber("TurnTurret Manual MaxAccel", kMaxTurnAcceleration);
    SmartDashboard.putNumber("TurnTurret Manual Tolerance", 0.5);

    pidAngVel = new PIDController(kPTurn, 0, kDTurn);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetType = originalTargetType;

    log.writeLog(false, "TurretTurnAngle", "initialize", "softLimitFwd", softLimitFwd, "softLimitRev", softLimitRev );
    // Do not execute if the turret is not calibrated
    encoderCalibrated = turret.isEncoderCalibrated();
    if (!encoderCalibrated) {
      log.writeLog(false, "TurretTurnAngle", "initialize", "turret not calibrated" );
      return;
    }
    
    feedbackUsingVision = false;

    if(fromShuffleboard) {
      target = SmartDashboard.getNumber("TurnTurret Manual Target Ang", 45);
      maxVel = SmartDashboard.getNumber("TurnTurret Manual MaxVel", 150);
      maxVel = MathUtil.clamp(Math.abs(maxVel), 0, kClampTurnVelocity);
      maxAccel = SmartDashboard.getNumber("TurnTurret Manual MaxAccel", kMaxTurnAcceleration);
      maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, kMaxTurnAcceleration);
      angleTolerance = SmartDashboard.getNumber("TurnTurret Manual Tolerance", 0.5);
      continualTracking = angleTolerance < 0;
      this.angleTolerance = Math.abs(angleTolerance);
    }
    // If constants were updated from Shuffleboard, then update PID
    pidAngVel.setPID(kPTurn, 0, kDTurn);
    pidAngVel.reset();

    startAngle = turret.getTurretPosition();

    switch (targetType) {
      case kRelative:
        targetRel = target;
        break;
      case kAbsolute:
        target = MathBCR.normalizeAngle(target);
        if (target>softLimitFwd) target = softLimitFwd;
        if (target<softLimitRev) target = softLimitRev;
        targetRel = target - startAngle;
        break;
      case kVisionOnScreen:
        if (piVisionHub.seesTarget()) {
          log.writeLog(false, "TurretTurnAngle", "initialize", "vision sees target" );
          targetRel = MathBCR.normalizeAngle(piVisionHub.getXOffset());
          piVisionHub.enableFastLogging(true);
        } else { 
          // no target is found; don't turn, just exit
          log.writeLog(false, "TurretTurnAngle", "initialize", "no target found - exiting" );
          targetRel = 0;
          // continualTracking = false;
          //targetType = TargetType.kRelative;
        }
        break;
      case kVisionScanLeft:
        if (piVisionHub.seesTarget()) {
          log.writeLog(false, "TurretTurnAngle", "initialize", "vision sees target" );
          targetRel = MathBCR.normalizeAngle(piVisionHub.getXOffset());
          piVisionHub.enableFastLogging(true);
        } else {
          log.writeLog(false, "TurretTurnAngle", "initialize", "no target found - scanning left" );
          target = softLimitRev;
          targetRel = target - startAngle;
        }
        break;
      case kVisionScanRight:
        if (piVisionHub.seesTarget()) {
          log.writeLog(false, "TurretTurnAngle", "initialize", "vision sees target" );
          targetRel = MathBCR.normalizeAngle(piVisionHub.getXOffset());
          piVisionHub.enableFastLogging(true);
        } else {
          log.writeLog(false, "TurretTurnAngle", "initialize", "no target found - scanning right" );
          target = softLimitFwd;
          targetRel = target - startAngle;
        }
        break;
    }

    // Prevent turret from wrapping
    if ( (startAngle + targetRel)>softLimitFwd )  {
      targetRel = softLimitFwd - startAngle;
      log.writeLog(false, "TurretTurnAngle", "initialize", "soft limit fwd set target angle", targetRel );
    }

    if ( (startAngle + targetRel)<softLimitRev )  {
      targetRel = softLimitRev - startAngle;
      log.writeLog(false, "TurretTurnAngle", "initialize", "soft limit rev set target angle", targetRel );
    }

    // Set initial and final states
    tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0); // initialize goal state (degrees to turn)
    tStateCurr = new TrapezoidProfileBCR.State(0.0, turret.getTurretVelocity()); // initialize initial state (relative turning, so assume initPos is 0 degrees)

    // Clamp acceleration for short turns
    if ( Math.abs(targetRel) <= kShortTurn) maxAccel = MathUtil.clamp(Math.abs(maxAccel), 0, kClampAccelShortTurn);

    // initialize velocity and accel limits
    tConstraints = new TrapezoidProfileBCR.Constraints(maxVel , maxAccel);
    // generate profile
    tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

    profileStartTime = System.currentTimeMillis(); // save starting time of profile
    currProfileTime = profileStartTime;

    log.writeLog(false, "TurretTurnAngle", "initialize", "Total Time", tProfile.totalTime(), "targetType", targetType, "StartAngleAbs", startAngle, "TargetAngleRel", targetRel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do not execute if the turret is not calibrated
    if (!encoderCalibrated) return;
    
    double pFF, pFB, pDB;  // variables for feed forward power, feedback power, and deadband power

    currProfileTime = System.currentTimeMillis();
    // currAngle is relative to the startAngle
    currAngle = turret.getTurretPosition() - startAngle;
    currVelocity = turret.getTurretVelocity();
    
    if (piVisionHub.seesTarget() && (targetType == TargetType.kVisionOnScreen || targetType == TargetType.kVisionScanLeft || targetType == TargetType.kVisionScanRight)) {  
      // If using vision and we see the goal, then update target angle to the location of the goal

      // Changed 9/25/2022:  Vision target angle has ~50msec lag, which caused problems when regenerating the trapezoid
      //    using the "latest" reported angle from the camera.  Instead of regenerating vision target from camera,
      //    just use the original target angle instead (and repeat the commend if needed to get the turret more accurate to the target).
      // targetRel = MathBCR.normalizeAngle(currAngle + piVisionHub.getXOffset());
      // tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0);
    } else if (!piVisionHub.seesTarget() && targetType == TargetType.kVisionScanLeft && Math.abs(softLimitRev - currAngle - startAngle) < 5) {
      // If using vision and scanning left and we don't see the target and we reach the left soft limit, then start scanning right
      targetType = TargetType.kVisionScanRight;
      target = softLimitFwd;
      targetRel = target - startAngle;
      tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0);
    } else if (!piVisionHub.seesTarget() && targetType == TargetType.kVisionScanRight && Math.abs(softLimitFwd - currAngle - startAngle) < 5) {
      // If using vision and scanning right and we don't see the target and we reach the right soft limit, then start scanning left
      targetType = TargetType.kVisionScanLeft;
      target = softLimitRev;
      targetRel = target - startAngle;
      tStateFinal = new TrapezoidProfileBCR.State(targetRel, 0.0);
    }

    timeSinceStart = (double)(currProfileTime - profileStartTime) * 0.001;
    tStateNow = tProfile.calculate(timeSinceStart);        // This is where the robot should be now
    tStateForecast = tProfile.calculate(timeSinceStart + tLagTurn);  // This is where the robot should be next cycle (or farther in the future if the robot has lag or backlash)

//    if(tProfile.isFinished(timeSinceStart) && (targetType == TargetType.kVisionOnScreen || targetType == TargetType.kVisionScanLeft || targetType == TargetType.kVisionScanRight)){  
    if( piVisionHub.seesTarget() && (targetType == TargetType.kVisionOnScreen || targetType == TargetType.kVisionScanLeft || targetType == TargetType.kVisionScanRight)
        && (Math.abs(targetRel - currAngle)<=5) ){  
      // If we completed the trapezoid profile and we are using vision, then
      // fine-tune angle using feedback based on live camera feedback
      feedbackUsingVision = true;
    }

    targetVel = tStateNow.velocity;
    targetAccel = tStateNow.acceleration;
    double forecastVel = tStateForecast.velocity;
    // double forecastAccel = MathUtil.clamp((forecastVel-targetVel)/tLagTurn, -maxAccel, maxAccel);    
    TrapezoidProfileBCR.State tStateFutureAccel = tProfile.calculate(timeSinceStart + .05);       // The acceleration tends to cause velocity overshoot after accel goes to 0.  So, use accel value 50ms in the future to avoid overshoot.
    double forecastAccel = tStateFutureAccel.acceleration;

    if(!tProfile.isFinished(timeSinceStart) && !feedbackUsingVision){
      // Feed-forward percent voltage to drive motors
      pFF = (forecastVel * kVTurn) + (forecastAccel * kATurn);
      // Normal feedback for following trapezoid profile
      pFB = MathUtil.clamp(pidAngVel.calculate(currVelocity, targetVel) + kITurn * (tStateNow.position - currAngle), -0.1, 0.1);
    } else {
      // We are past the end of the trapezoid profile
      forecastVel = 0.0;
      forecastAccel = 0.0;
      pFF = 0.0;

      if (targetType == TargetType.kVisionOnScreen || targetType == TargetType.kVisionScanLeft || targetType == TargetType.kVisionScanRight) {
        // Live camera feedback
        // 9/25/2022:  Don't use live camera feedback, due to ~50msec lag in camera position reporting.
        // pFB = kITurnEnd * Math.signum( MathBCR.normalizeAngle(piVisionHub.getXOffset()) );
        // pFB = kITurnEnd * MathBCR.normalizeAngle(piVisionHub.getXOffset());

        pFB = kITurnEnd * ( targetRel - currAngle );
      } else {
        // TODO Tune this better?  use adaptive minimum speed?
        pFB = kITurnEnd * ( targetRel - currAngle );
      }
    }
    
    // Use kS to account for dead band
    pDB = kSTurn * Math.signum(pFF + pFB);

    turret.setPercentOutput(+pFF + pFB + pDB);

    if (regenerate) {
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, currVelocity);
      // tStateCurr = new TrapezoidProfileBCR.State(currAngle, targetVel);  
      // tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateCurr);

      // When regenerating using tStateNow, the profile only changes if the final state has moved (such as due to vision)
      tProfile = new TrapezoidProfileBCR(tConstraints, tStateFinal, tStateNow);
      profileStartTime = currProfileTime;
    }

    log.writeLog(false, "TurretTurnAngle", "profile", "target", targetRel, 
      "posT", tStateNow.position, "velT", targetVel, "accT", targetAccel,
      "posF", tStateForecast.position, "velF", forecastVel, "accF", forecastAccel,
      "posA", currAngle, "velA", currVelocity, "pFF", pFF, "pFB", pFB, "pTotal", pFF+pFB+pDB,
      "counter", accuracyCounter
      , "pi x", piVisionHub.getXOffset()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stopMotor();
    
    if (targetType == TargetType.kVisionOnScreen || targetType == TargetType.kVisionScanLeft || targetType == TargetType.kVisionScanRight) {
      piVisionHub.enableFastLogging(false);
    }
    
    log.writeLog(false, "TurretTurnAngle", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Do not execute if the turret is not calibrated
    if (!encoderCalibrated) return true;
    if (continualTracking) return false;

    if (((targetType == TargetType.kAbsolute || targetType == TargetType.kRelative) && Math.abs(targetRel - currAngle) < angleTolerance) ||
    ((targetType == TargetType.kVisionOnScreen || targetType == TargetType.kVisionScanLeft || targetType == TargetType.kVisionScanRight) 
    && ((piVisionHub.seesTarget() && Math.abs(piVisionHub.getXOffset()) < angleTolerance) || (!piVisionHub.seesTarget() && feedbackUsingVision)))) {
    // if (Math.abs(targetRel - currAngle) < angleTolerance) {
      accuracyCounter++;
      log.writeLog(false, "TurretTurnAngle", "WithinTolerance", "Target Ang", targetRel, "Actual Ang", currAngle, 
        "piVisionHub Xoff", piVisionHub.getXOffset(), 
        "Counter", accuracyCounter);
    } else {
      accuracyCounter = 0;
    }

    if(accuracyCounter >= 5) {
      return true;
    }
    return false;
  }
}