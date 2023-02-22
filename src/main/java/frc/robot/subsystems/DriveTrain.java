/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import frc.robot.utilities.*;
import static frc.robot.Constants.Ports.*;
import static frc.robot.Constants.RobotConstants.*;

// Vision imports
import frc.robot.subsystems.PhotonCameraWrapper;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

import static frc.robot.Constants.DriveConstants.*;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX leftMotor1;
  private final WPI_TalonFX leftMotor2;
  private final WPI_TalonFX rightMotor1;
  private final WPI_TalonFX rightMotor2;

  private final DifferentialDrive diffDrive;
  //private final DifferentialDriveOdometry odometry;

  private double leftEncoderZero = 0;
  private double rightEncoderZero = 0;

  private final AHRS ahrs;
  private double yawZero = 0;

  private FileLog log;
  private TemperatureCheck tempCheck;

  // variable to store vision camera
  private PhotonCameraWrapper camera;
  private DifferentialDrivePoseEstimator poseEstimator; 
  private Field2d field = new Field2d();    // Field to dispaly on Shuffleboard
  
  // variables to help calculate angular velocity for turnGyro
  private double prevAng; // last recorded gyro angle
  private double currAng; // current recorded gyro angle
  private double prevTime; // last time gyro angle was recorded
  private double currTime; // current time gyro angle is being recorded
  private double angularVelocity;  // Robot angular velocity in degrees per second
  private LinearFilter lfRunningAvg = LinearFilter.movingAverage(4); //calculate running average to smooth quantization error in angular velocity calc
  
  public DriveTrain(Field fieldUtil, FileLog log, TemperatureCheck tempCheck) {
    this.log = log; // save reference to the fileLog
    this.camera = new PhotonCameraWrapper(fieldUtil);
    this.tempCheck = tempCheck;

    // configure navX
    AHRS gyro = null;
		try {
      gyro = new AHRS(SerialPort.Port.kUSB);
      // gyro.zeroYaw();   // *** Do not zero the gyro hardware!  The hardware zeros asynchronously from this thread, so an immediate read-back of the gyro may not yet be zeroed.
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    ahrs = gyro;
    
    // configure motors
    leftMotor1 = new WPI_TalonFX(CANLeftDriveMotor1);
    leftMotor2 = new WPI_TalonFX(CANLeftDriveMotor2);
    rightMotor1 = new WPI_TalonFX(CANRightDriveMotor1);
    rightMotor2 = new WPI_TalonFX(CANRightDriveMotor2);

    leftMotor1.configFactoryDefault();
    leftMotor2.configFactoryDefault();
    rightMotor1.configFactoryDefault();
    rightMotor2.configFactoryDefault();

    leftMotor2.set(ControlMode.Follower, CANLeftDriveMotor1);
    rightMotor2.set(ControlMode.Follower, CANRightDriveMotor1);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    leftMotor1.setInverted(false);
    leftMotor2.setInverted(false);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

    setDriveModeCoast(false);

    leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    
    leftMotor1.setSensorPhase(false);
    rightMotor1.setSensorPhase(false);

    leftMotor1.configNeutralDeadband(0.0);
    leftMotor2.configNeutralDeadband(0.0);
    rightMotor1.configNeutralDeadband(0.0);
    rightMotor2.configNeutralDeadband(0.0);

    leftMotor1.configVoltageCompSaturation(compensationVoltage);
    leftMotor2.configVoltageCompSaturation(compensationVoltage);
    rightMotor1.configVoltageCompSaturation(compensationVoltage);
    rightMotor2.configVoltageCompSaturation(compensationVoltage);

    // Change number of samples in the rolling average for voltage compensation (default = 12?)
    // leftMotor1.configVoltageMeasurementFilter(4);
    // leftMotor2.configVoltageMeasurementFilter(4);
    // rightMotor1.configVoltageMeasurementFilter(4);
    // rightMotor2.configVoltageMeasurementFilter(4);

    setVoltageCompensation(true);
    setOpenLoopRampLimit(true);

    // Set stator current limits to reduce overheating
    // StatorCurrentLimitConfiguration currLimitCfg = new StatorCurrentLimitConfiguration(true, 50, 60, 0.05);     // Units = boolean, Amps, Amps, seconds
    // leftMotor1.configStatorCurrentLimit(currLimitCfg);
    // leftMotor2.configStatorCurrentLimit(currLimitCfg);
    // rightMotor1.configStatorCurrentLimit(currLimitCfg);
    // rightMotor2.configStatorCurrentLimit(currLimitCfg);

    // create the differential drive AFTER configuring the motors
    diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);
    diffDrive.setDeadband(0.0);
    
    zeroLeftEncoder();
    zeroRightEncoder();
    zeroGyroRotation();

    

    // Sets initial position to (0,0) facing 0 degrees
    //odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroRotation()), 0, 0);

    poseEstimator = new DifferentialDrivePoseEstimator(kDriveKinematics, Rotation2d.fromDegrees(getGyroRotation()), 
       0,0, new Pose2d(0, 0, Rotation2d.fromDegrees(0)) );


    // initialize angular velocity variables
    prevAng = getGyroRaw();
    currAng = getGyroRaw();
    prevTime = System.currentTimeMillis();
    currTime = System.currentTimeMillis();
    lfRunningAvg.reset();

    // Record status frame periods
    log.writeLogEcho(true, "Drive", "Init", "Status Frame 1 period", leftMotor1.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General),
                     "Status Frame 4 period", leftMotor1.getStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat));

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Drive kV Linear", kVLinear); // Linear coefficients
    SmartDashboard.putNumber("Drive kA Linear", kALinear);
    SmartDashboard.putNumber("Drive kS Linear", kSLinear);
    SmartDashboard.putNumber("Drive kP Linear", kPLinear);
    SmartDashboard.putNumber("Drive kI Linear", kILinear);
    SmartDashboard.putNumber("Drive kD Linear", kDLinear);
    SmartDashboard.putNumber("Drive kAng Linear", kAngLinear);

    SmartDashboard.putNumber("Drive kV Angular", kVAngular); // Angular coefficients
    SmartDashboard.putNumber("Drive kA Angular", kAAngular);
    SmartDashboard.putNumber("Drive kS Angular", kSAngular);
    SmartDashboard.putNumber("Drive kP Angular", kPAngular);
    SmartDashboard.putNumber("Drive kI Angular", kIAngular);
    SmartDashboard.putNumber("Drive kD Angular", kDAngular);
    SmartDashboard.putNumber("Drive tLag Angular", tLagAngular);
  }

  /**
   * Tank drive method for differential drive platform. The calculated 
   * values will be squared to decrease sensitivity at low speeds.
   * @param leftPercent The robot's left side percent along the X axis [-1.0..1.0]. Forward is positive.
   * @param rightPercent The robot's right side percent along the X axis [-1.0..1.0]. Forward is positive.
   */
  public void tankDrive(double leftPercent, double rightPercent) {
    diffDrive.tankDrive(leftPercent, rightPercent, true);
  }

  /**
   * Tank drive method for differential drive platform.
   * @param leftPercent  The robot's left side percent along the X axis [-1.0..1.0]. Forward is positive.
   * @param rightPercent The robot's right side percent along the X axis [-1.0..1.0]. Forward is positive.
   * @param squareInputs If set, decreases the input sensitivity at low speeds.
   */
  public void tankDrive(double leftPercent, double rightPercent, boolean squareInputs) {
    diffDrive.tankDrive(leftPercent, rightPercent, squareInputs);
  }

  /**
   * Stops motors by calling tankDrive(0, 0).
   */
  public void stop() {
    tankDrive(0.0, 0.0);
  }

  /**
   * Call when not using arcade drive or tank drive to turn motors to
   * ensure that motor will not cut out due to differential drive safety.
   */
  public void feedTheDog() {
    diffDrive.feed();
  }

  /**
   * Sets the percent output on the Left motor.
   * <P>This method must be called periodically (such as in a command's execute() method) to prevent
   * the motors from cutting out due to differential drive safety.
   * @param percent percent output (+1 = forward, -1 = reverse)
   */
  public void setLeftMotorOutput(double percent) {
    leftMotor1.set(ControlMode.PercentOutput, percent);
    feedTheDog();
  }

  /**
   * Sets the percent output on the Right motor.
   * <P>This method must be called periodically (such as in a command's execute() method) to prevent
   * the motors from cutting out due to differential drive safety.
   * @param percent percent output (+1 = forward, -1 = reverse)
   */
  public void setRightMotorOutput(double percent) {
    rightMotor1.set(ControlMode.PercentOutput, percent);
    feedTheDog();
  }

  /**
   * Drive the robot using arcade controls
   * @param speedPct
   * @param rotation
   */
  public void arcadeDrive(double speedPct, double rotation) {
    double maxRotation = 0.25;
    double absSpeed = Math.abs(speedPct);
    double rotClamp = MathUtil.clamp(rotation, Math.min(-absSpeed, -maxRotation), Math.max(absSpeed, maxRotation));
    diffDrive.arcadeDrive(speedPct, rotClamp, false);    // minimize how fast turn operated from joystick
  }

  /**
   * @param turnOn true = turn on voltage compensation, false = turn off voltage compensation
   */
  public void setVoltageCompensation(boolean turnOn) {
    leftMotor1.enableVoltageCompensation(turnOn);
    leftMotor2.enableVoltageCompensation(turnOn);
    rightMotor1.enableVoltageCompensation(turnOn);
    rightMotor2.enableVoltageCompensation(turnOn);
  }
  
  /**
   * @param turnOn true = turn on open loop ramp rate limit, false = turn off open loop ramp rate limit
   */
  public void setOpenLoopRampLimit(boolean turnOn) {
    double ramp = turnOn ? 0.60 : 0.0;     // Was 0.4, but robot tips easily
    leftMotor1.configOpenloopRamp(ramp);
    leftMotor2.configOpenloopRamp(ramp);
    rightMotor1.configOpenloopRamp(ramp);
    rightMotor2.configOpenloopRamp(ramp);
  }

  /**
   * @return left encoder position, in ticks
   */
  public double getLeftEncoderRaw() {
    return leftMotor1.getSelectedSensorPosition(0);
  }

  /**
   * @return right encoder position, in ticks
   */
  public double getRightEncoderRaw() {
    return rightMotor1.getSelectedSensorPosition(0);
  }

  /**
   * @return left encoder velocity, in ticks per 100ms (+ = forward)
   */
  public double getLeftEncoderVelocityRaw() {
    return leftMotor1.getSelectedSensorVelocity(0);
  }

  /**
   * @return right encoder velocity, in ticks per 100ms (+ = forward)
   */
  public double getRightEncoderVelocityRaw() {
    return rightMotor1.getSelectedSensorVelocity(0);
  }

  /**
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setDriveModeCoast(boolean setCoast) {
    if (setCoast) {
      leftMotor1.setNeutralMode(NeutralMode.Coast);
      leftMotor2.setNeutralMode(NeutralMode.Coast);
      rightMotor1.setNeutralMode(NeutralMode.Coast);
      rightMotor2.setNeutralMode(NeutralMode.Coast);
    } else {
      leftMotor1.setNeutralMode(NeutralMode.Brake);
      leftMotor2.setNeutralMode(NeutralMode.Brake);
      rightMotor1.setNeutralMode(NeutralMode.Brake);
      rightMotor2.setNeutralMode(NeutralMode.Brake);
    }
  }

  /**
	 * Zero the left encoder position in software.
	 */
  public void zeroLeftEncoder() {
    leftEncoderZero = getLeftEncoderRaw();
  }

  /**
	 * Zero the right encoder position in software.
	 */
  public void zeroRightEncoder() {
    rightEncoderZero = getRightEncoderRaw();
  }

  /**
	 * Get the position of the left encoder since last zeroLeftEncoder().
	 * @return encoder position, in ticks
   */
  public double getLeftEncoderTicks() {
    return getLeftEncoderRaw() - leftEncoderZero;
  }

  /**
	 * Get the position of the right encoder since last zeroRightEncoder().
	 * @return encoder position, in ticks
	 */
  public double getRightEncoderTicks() {
    return getRightEncoderRaw() - rightEncoderZero;
  }

  /**
   * @param ticks encoder ticks
   * @return parameter encoder ticks converted to equivalent inches
   */
  public static double encoderTicksToInches(double ticks) {
    return ticks / ticksPerInch;
  }

  /**
   * @param inches inches
   * @return parameter inches converted to equivalent encoder ticks
   */
  public static double inchesToEncoderTicks(double inches) {
    return inches * ticksPerInch;
  }

  /**
   * @return left encoder position, in inches
   */
  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  /**
   * @return right encoder position, in inches
   */
  public double getRightEncoderInches() {
    return encoderTicksToInches(getRightEncoderTicks());
  }

  /**
   * @return average between left and right encoders, in inches
   */
  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
  }

  /**
   * @return left encoder velocity, in inches per second (+ = forward)
   */
  public double getLeftEncoderVelocity() {
    return encoderTicksToInches(getLeftEncoderVelocityRaw()) * 10;
  }

  /**
   * @return right encoder velocity, in inches per second (+ = forward)
   */
  public double getRightEncoderVelocity() {
    return encoderTicksToInches(getRightEncoderVelocityRaw()) * 10;
  }

  /**
   * @return average velocity, in inches per second
   */
  public double getAverageEncoderVelocity(){
    return (getRightEncoderVelocity() + getLeftEncoderVelocity()) / 2;
  }

  /**
   * Gets the raw gyro angle (can be greater than 360).
   * Angle is negated from the gyro, so that + = left and - = right
   * @return raw gyro angle, in degrees.
   */
  public double getGyroRaw() {
    return -ahrs.getAngle();
  }

  /**
	 * Zero the gyro position in software to the current angle.
	 */
	public void zeroGyroRotation() {
    yawZero = getGyroRaw(); // set yawZero to gyro angle
  }
  
  /**
	 * Zero the gyro position in software against a specified angle.
	 * @param currentHeading current robot angle compared to the zero angle
	 */
	public void zeroGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
  }

  /**
	 * @return gyro angle from 180 to -180, in degrees (postitive is left, negative is right)
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert to (-180, 180]
		angle = normalizeAngle(angle);
		return angle;
  }

  /**
   * Verifies if Gyro is still reading
   * @return true = gryo is connected to Rio
   */
  public boolean isGyroReading() {
    return ahrs.isConnected();
  }

  /**
   * @return gyro angular velocity (with some averaging to reduce noise), in degrees per second.
   * Positive is turning left, negative is turning right.
   */
  public double getAngularVelocity () {
    return angularVelocity;
  }

  /**
   * @return angular velocity from motor velocity readings (NOT from gyro)
   * Positive is turning left, negative is turning right.
   */
  public double getAngularVelocityFromWheels () {
    return ((getRightEncoderVelocity() - getLeftEncoderVelocity()) / 2) * wheelInchesToGyroDegrees;
  }

  /**
	 * Converts input angle to a number between -179.999 and +180.0.
	 * @return normalized angle
	 */
	public double normalizeAngle(double angle) {
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
  }

  /**
   * Set up PID parameters for the drive train Talons
   * @param kP Proportional term
   * @param kI Integral term 
   * @param kD Differential term
   * @param kF Feed forward term (multiplied by setpoint)
   */
  public void setTalonPIDConstants(double kP, double kI, double kD, double kF) {
    leftMotor1.config_kP(0, kP);
    leftMotor1.config_kI(0, kI);
    leftMotor1.config_kD(0, kD);
    leftMotor1.config_kF(0, kF);

    rightMotor1.config_kP(0, kP);
    rightMotor1.config_kI(0, kI);
    rightMotor1.config_kD(0, kD);
    rightMotor1.config_kF(0, kF);

    SmartDashboard.putNumber("Drive kP Linear", kP);
    SmartDashboard.putNumber("Drive kI Linear", kI);
    SmartDashboard.putNumber("Drive kD Linear", kD);

    leftMotor1.selectProfileSlot(0, 0);
    rightMotor1.selectProfileSlot(0, 0);
  }

  /**
   * Resets the Talon PIDs.  Use this when re-starting the PIDs.
   */
  public void resetTalonPIDs() {
    leftMotor1.setIntegralAccumulator(0);
    rightMotor1.setIntegralAccumulator(0);
  }

  /**
   * Sets the left Talon to velocity closed-loop control mode with target velocity and feed-forward constant.
   * <P>This method must be called periodically (such as in a command's execute() method) to prevent
   * the motors from cutting out due to differential drive safety.
   * @param targetVel Target velocity, in inches per second
   * @param aFF Feed foward term to add to the contorl loop (-1 to +1)
   */
  public void setLeftTalonPIDVelocity(double targetVel, double aFF) {
    leftMotor1.set(ControlMode.Velocity, 
      targetVel * ticksPerInch / 10.0, DemandType.ArbitraryFeedForward, aFF);
    feedTheDog();
  }

  /**
   * Sets the right Talon to velocity closed-loop control mode with target velocity and feed-forward constant.
   * <P>This method must be called periodically (such as in a command's execute() method) to prevent
   * the motors from cutting out due to differential drive safety.
   * @param targetVel Target velocity, in inches per second
   * @param aFF Feed foward term to add to the contorl loop (-1 to +1)
   * @param reverseRight True = reverse velocity and FF term for right Talon
   */
  public void setRightTalonPIDVelocity(double targetVel, double aFF, boolean reverseRight) {
    int direction = (reverseRight) ? 1 : -1;
    rightMotor1.set(ControlMode.Velocity, 
      targetVel*direction  * ticksPerInch / 10.0, DemandType.ArbitraryFeedForward, aFF*direction);
    feedTheDog();
  }

  public double getLeftOutputVoltage() {
    return leftMotor1.getMotorOutputVoltage();
  }

  public double getLeftOutputVoltage2() {
    return leftMotor2.getMotorOutputVoltage();
  }

  public double getLeftBusVoltage() {
    return leftMotor1.getBusVoltage();
  }

  public double getLeftOutputPercent() {
    return leftMotor1.getMotorOutputPercent();
  }

  public double getLeftStatorCurrent() {
    return leftMotor1.getStatorCurrent();
  }

  public double getLeftStatorCurrent2() {
    return leftMotor2.getStatorCurrent();
  }

  public double getRightOutputVoltage() {
    return rightMotor1.getMotorOutputVoltage();
  }

  public double getRightOutputVoltage2() {
    return rightMotor2.getMotorOutputVoltage();
  }

  public double getRightBusVoltage() {
    return rightMotor1.getBusVoltage();
  }

  public double getRightTemp() {
    return rightMotor1.getTemperature();
  }

  public double getRightOutputPercent() {
    return rightMotor1.getMotorOutputPercent();
  }

  public double getRightStatorCurrent() {
    return rightMotor1.getStatorCurrent();
  }

  public double getRightStatorCurrent2() {
    return rightMotor2.getStatorCurrent();
  }

  public double getTalonLeftClosedLoopError() {
    return leftMotor1.getClosedLoopError();
  }

  public double getTalonLeftClosedLoopTarget() {
    return leftMotor1.getClosedLoopTarget();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Update robot odometry
    double degrees = getGyroRotation();
    // double leftMeters = Units.inchesToMeters(getLeftEncoderInches());
    // double rightMeters = Units.inchesToMeters(getRightEncoderInches());
    //odometry.update(Rotation2d.fromDegrees(degrees), leftMeters, rightMeters);

    // update 
    updateOdometry();

    // save current angle and time for calculating angVel
    currAng = getGyroRaw();
    currTime = System.currentTimeMillis();
 
    // calculate angVel in degrees per second
    angularVelocity =  lfRunningAvg.calculate( (currAng - prevAng) / (currTime - prevTime) * 1000 );
    
    if(log.getLogRotation() == log.DRIVE_CYCLE) {
      updateDriveLog(false);

      if(!isGyroReading()) {
        RobotPreferences.recordStickyFaults("Gyro", log);
      }

      // read PID coefficients from SmartDashboard
      kVLinear = SmartDashboard.getNumber("Drive kV Linear", kVLinear);
      kALinear = SmartDashboard.getNumber("Drive kA Linear", kALinear);
      kSLinear = SmartDashboard.getNumber("Drive kS Linear", kSLinear);
      kPLinear = SmartDashboard.getNumber("Drive kP Linear", kPLinear);
      kILinear = SmartDashboard.getNumber("Drive kI Linear", kILinear);
      kDLinear = SmartDashboard.getNumber("Drive kD Linear", kDLinear);
      kAngLinear = SmartDashboard.getNumber("Drive kAng Linear", kAngLinear);

      kVAngular = SmartDashboard.getNumber("Drive kV Angular", kVAngular);
      kAAngular = SmartDashboard.getNumber("Drive kA Angular", kAAngular);
      kSAngular = SmartDashboard.getNumber("Drive kS Angular", kSAngular);
      kPAngular = SmartDashboard.getNumber("Drive kP Angular", kPAngular);
      kIAngular = SmartDashboard.getNumber("Drive kI Angular", kIAngular);
      kDAngular = SmartDashboard.getNumber("Drive kD Angular", kDAngular);
      tLagAngular = SmartDashboard.getNumber("Drive tLag Angular", tLagAngular);
       
      // Update data on SmartDashboard
      SmartDashboard.putNumber("Drive High Temp", Math.max(rightMotor1.getTemperature(), Math.max(rightMotor2.getTemperature(), Math.max(leftMotor1.getTemperature(), leftMotor2.getTemperature()))));
      SmartDashboard.putNumber("Drive Right Raw", getRightEncoderRaw());
      SmartDashboard.putNumber("Drive Left Raw", getLeftEncoderRaw());
      SmartDashboard.putNumber("Drive Right Enc", getRightEncoderInches());
      SmartDashboard.putNumber("Drive Left Enc", getLeftEncoderInches());
      SmartDashboard.putNumber("Drive Average Dist in Meters", Units.inchesToMeters(getAverageDistance()));
      SmartDashboard.putNumber("Drive Left Velocity", getLeftEncoderVelocity());
      SmartDashboard.putNumber("Drive Right Velocity", getRightEncoderVelocity());
      SmartDashboard.putNumber("Drive Gyro Rotation", degrees);
      SmartDashboard.putNumber("Drive AngVel", angularVelocity);
      SmartDashboard.putNumber("Drive Raw Gyro", getGyroRaw());
      SmartDashboard.putBoolean("Drive isGyroReading", isGyroReading());
      SmartDashboard.putNumber("Drive Pitch", ahrs.getRoll());
      
      // position from odometry (helpful for autos)
      var translation =  poseEstimator.getEstimatedPosition().getTranslation();;
      SmartDashboard.putNumber("Drive Odometry X",translation.getX());
      SmartDashboard.putNumber("Drive Odometry Y",translation.getY());

      //Values for bugfixing
      SmartDashboard.putNumber("Drive Motor Temp", leftMotor1.getTemperature());
      SmartDashboard.putNumber("Drive L2 Temp", leftMotor2.getTemperature());
      SmartDashboard.putNumber("Drive R1 Temp", rightMotor1.getTemperature());
      SmartDashboard.putNumber("Drive R2 Temp", rightMotor2.getTemperature());
      SmartDashboard.putNumber("Drive Bus Volt", leftMotor1.getBusVoltage());
    }

    // save current angVel values as previous values for next calculation
    prevAng = currAng;
    prevTime = currTime; 
  }

  /**
   * Get current robot location and facing on the field
   * @return current robot pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the robot pose on the field to the given location and rotation
   * <p>Note:  This method resets the encoders to 0 and sets the gyro
   * to the current robot rotation.
   * <p>Robot X: 0 = middle of robot, wherever the robot starts auto mode (+=away from our drivestation)
   * <p>Robot Y: 0 = middle of robot, wherever the robot starts auto mode (+=left when looking from our drivestation)
   * <p>Robot angle: 0 = facing away from our drivestation
   * @param robotPoseInMeters Current robot pose, in meters
   */
  public void resetPose(Pose2d robotPoseInMeters) {
    zeroLeftEncoder();
    zeroRightEncoder();
    zeroGyroRotation(robotPoseInMeters.getRotation().getDegrees());
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyroRotation()), 0, 0, robotPoseInMeters);
  }

  /**
   * @return wheel speeds, in meters per second
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(Units.inchesToMeters(getLeftEncoderVelocity()), Units.inchesToMeters(getRightEncoderVelocity()));
  }

  /**
   * Writes information about the drive train to the filelog
   * @param logWhenDisabled true will log when disabled, false will discard the string
   */
  public void updateDriveLog(boolean logWhenDisabled) {
    var translation = poseEstimator.getEstimatedPosition().getTranslation();
    log.writeLog(logWhenDisabled, "Drive", "Update Variables", 
      "L1 Volts", leftMotor1.getMotorOutputVoltage(), "L2 Volts", leftMotor2.getMotorOutputVoltage(),
      "L1 Amps", leftMotor1.getSupplyCurrent(), "L2 Amps", leftMotor2.getSupplyCurrent(),
      "L1 Temp",leftMotor1.getTemperature(), "L2 Temp",leftMotor2.getTemperature(),
      "R1 Volts", rightMotor1.getMotorOutputVoltage(), "R2 Volts", rightMotor2.getMotorOutputVoltage(),
      "R1 Amps", rightMotor1.getSupplyCurrent(), "R2 Amps", rightMotor2.getSupplyCurrent(), 
      "R1 Temp",rightMotor1.getTemperature(), "R2 Temp",rightMotor2.getTemperature(),
      "Left Inches", getLeftEncoderInches(), "L Vel", getLeftEncoderVelocity(),
      "Right Inches", getRightEncoderInches(), "R Vel", getRightEncoderVelocity(),
      "Gyro Angle", getGyroRotation(), "RawGyro", getGyroRaw(), 
      "Gyro Velocity", angularVelocity, 
      "Odometry X", translation.getX(), "Odometry Y", translation.getY(),
      "Pitch", ahrs.getRoll()
      );
  }

  /**
   * Update TemperatureCheck utility with motors that are and are not overheating.
   * 
   * <p>NOTE: !!!!!
   * This code was not used last year.  It may need debugging before it is used!
   */
  public void updateOverheatingMotors() {
    if (leftMotor1.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveLeft1");
    if (leftMotor2.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveLeft2");
    
    if (rightMotor1.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveRight1");
    if (rightMotor2.getTemperature() >= temperatureCheck)
      tempCheck.recordOverheatingMotor("DriveRight2");


    if (leftMotor1.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveLeft1");
    if (leftMotor1.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveLeft2");

    if (rightMotor1.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveRight1");
    if (rightMotor2.getTemperature() < temperatureCheck)
      tempCheck.notOverheatingMotor("DriveRight2");
  }

  public void updateOdometry() {
    poseEstimator.update(Rotation2d.fromDegrees(getGyroRotation()),  Units.inchesToMeters(getLeftEncoderInches()), Units.inchesToMeters(getRightEncoderInches()));

    Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        // only updates odometry if close enough 
        // TODO change how it decides if it's too far
        if (camPose.estimatedPose.getX() < 3.3) {
          poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);

          // field.setRobotPose(camPose.estimatedPose.toPose2d());
        }
    }
    
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }    
}
