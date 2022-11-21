/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class DriveCalibrate extends CommandBase {
  private DriveTrain driveTrain;
  private FileLog log;
  private double maxPctOut, rampTime, rampRate, signLeft, signRight;
  private double motorPctOut, gyroStart;
  private final Timer timer = new Timer();

  public enum CalibrateMode {
    kStraight(0),
    kTurnLeft(1),
    kTurnRight(2);

    @SuppressWarnings({"MemberName", "PMD.SingularField"})
    public final int value;

    CalibrateMode(int value) { this.value = value; }
  }

  /**
   * Ramp velocity of wheels for calibration.  Either drives straight or turns in place while logging the drive telemetry.
    * @param maxPctOut  Absolute value 0 to 1.  Power during ramp is clamped to this value.
    * @param rampTime Time in seconds for ramp
    * @param rampRate Ramp rate in pctOut/second 
    * @param mode kStraight, kTurnLeft, or kTurnRight
    * @param driveTrain
    * @param log
    */
  public DriveCalibrate(double maxPctOut, double rampTime, double rampRate, CalibrateMode mode, DriveTrain driveTrain, FileLog log) {
    this.maxPctOut = Math.abs(maxPctOut);
    this.driveTrain = driveTrain;
    this.log = log;
    this.rampTime = Math.abs(rampTime);
    this.rampRate = Math.abs(rampRate);

    switch (mode) {
      case kStraight:
        signLeft = 1.0;
        signRight = 1.0;
        break;
      case kTurnLeft:
        signLeft = -1.0;
        signRight = 1.0;
        break;
      case kTurnRight:
        signLeft = 1.0;
        signRight = -1.0;
    }
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    motorPctOut = 0;
    gyroStart = driveTrain.getGyroRotation();
    driveTrain.setOpenLoopRampLimit(false);

    log.writeLog(false, "DriveCalibrate", "init", "maxPctOut", maxPctOut, "rampTime", rampTime, "rampRate", rampRate, "signLeft", signLeft, "signRight", signRight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curTime = timer.get();

    log.writeLog(false, "DriveCalibrate", "execute", "time", curTime, 
      "pctOut", motorPctOut,
      "VbusLeft", driveTrain.getLeftBusVoltage(), "VbusRight", driveTrain.getRightBusVoltage(),
      "Vleft1", driveTrain.getLeftOutputVoltage(), "Vleft2", driveTrain.getLeftOutputVoltage2(), 
      "VRight1", driveTrain.getRightOutputVoltage(), "VRight2", driveTrain.getRightOutputVoltage2(),
      "Ileft1", driveTrain.getLeftStatorCurrent(), "Ileft2", driveTrain.getLeftStatorCurrent2(), 
      "IRight1", driveTrain.getRightStatorCurrent(), "IRight2", driveTrain.getRightStatorCurrent2(),
      "gyro", driveTrain.normalizeAngle(driveTrain.getGyroRotation()-gyroStart),
      "gyro vel", driveTrain.getAngularVelocity(),
      "ang vel from wheels", driveTrain.getAngularVelocityFromWheels(),
      "Lpos", driveTrain.getLeftEncoderInches(), "Lvel", driveTrain.getLeftEncoderVelocity(),
      "Rpos", driveTrain.getRightEncoderInches(), "Rvel", driveTrain.getRightEncoderVelocity()
    );

    motorPctOut = MathUtil.clamp( curTime*rampRate, -maxPctOut, maxPctOut);

    driveTrain.setLeftMotorOutput(signLeft*motorPctOut);
    driveTrain.setRightMotorOutput(signRight*motorPctOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveTrain.setOpenLoopRampLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(rampTime);
  }
}
