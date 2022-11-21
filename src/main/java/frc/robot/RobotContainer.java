// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.TargetType;
import frc.robot.Constants.UptakeConstants;
import frc.robot.Constants.BallColor;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.commands.*;
import frc.robot.commands.DriveFollowTrajectory.PIDType;
import frc.robot.commands.DriveCalibrate.CalibrateMode;
import frc.robot.commands.ShooterSetVelocity.InputMode;
import frc.robot.commands.commandGroups.*;
import frc.robot.subsystems.*;
import frc.robot.triggers.*;
import frc.robot.utilities.*;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot key utilities
  private final FileLog log = new FileLog("G6");
  private final TemperatureCheck tempCheck = new TemperatureCheck(log);
  private final PowerDistribution powerdistribution = new PowerDistribution(Ports.CANPowerDistHub, ModuleType.kRev);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
  private final AllianceSelection allianceSelection = new AllianceSelection(log);

  // Define robot subsystems  
  private final DriveTrain driveTrain = new DriveTrain(log, tempCheck);
  private final Shooter shooter = new Shooter(log);
  private final Feeder feeder = new Feeder("Feeder", log);
  private final Uptake uptake = new Uptake("Uptake", allianceSelection, log);
  private final IntakeFront intakeFront = new IntakeFront(log);
  private final Climber climber = new Climber("Climber", log);
  // private final Intake intakeRear = new Intake("Intake-Rear", Ports.CANIntakeRear, Ports.SolIntakeRearFwd, Ports.SolIntakeRearRev, log);
  private final Turret turret = new Turret(log);
  private final PiVisionHub pivisionhub = new PiVisionHub(powerdistribution, log); //Pi ip: 10.2.94.21S
  private final LimeLight limeLightFront = new LimeLight("limelight-front", log);
  // private final LimeLight limeLightRear = new LimeLight("limelight-rear", log);

  // Define final utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, log);

  // Define controllers
  // private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  // private final XboxController xbC = new XboxController(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private boolean rumbling = false;
  private boolean sensorConfigured = false;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureButtonBindings(); // configure button bindings
    configureShuffleboard(); // configure shuffleboard

    // don't configure triggers here as they interfere with autos. do it in teleopinit
    //configureSensorTriggers();

    driveTrain.setDefaultCommand(new DriveWithJoystickArcade(driveTrain, leftJoystick, rightJoystick, log));
  }

  /**
   * Configures any sensor triggers for the robot
   */
  private void configureSensorTriggers() {
    if (sensorConfigured == false) {
    
      Trigger colorSensorTrigger = new Trigger(() -> uptake.isBallAtColorSensor() && DriverStation.isTeleop());
      // colorSensorTrigger.whenActive(new UptakeSortBall(intakeFront, uptake, feeder, xboxController, log), false);
      colorSensorTrigger.onTrue(new UptakeSortBall(intakeFront, uptake, feeder, xboxController, log));

      // Trigger ejectSensorTrigger = new Trigger(() -> uptake.isBallInEjector());
      // ejectSensorTrigger.whenActive(new UptakeEjectTrigger(uptake, log));

      // Trigger feederSensorTrigger = new Trigger(() -> feeder.isBallPresent());
      // feederSensorTrigger.whenActive(new FeederSensorTrigger(feeder, log));
      
      sensorConfigured = true;
      log.writeLog(false , "Sensor Trigger", "Sensors Configured");
    } else {
      log.writeLog(false , "Sensor Trigger", "Sensors Already Configured");
    }

  }

  /**
   * Define Shuffleboard mappings.
   */
  private void configureShuffleboard() {

    // display sticky faults
    RobotPreferences.showStickyFaults();
    SmartDashboard.putData("Clear Sticky Faults", new StickyFaultsClear(log));

    // display overheating motors
    tempCheck.displayOverheatingMotors();

    // XBox rumble
    SmartDashboard.putData("XBox Rumble 1", new XboxRumble(0.5, 0.25, 1, xboxController, log));
    SmartDashboard.putData("XBox Rumble 2", new XboxRumble(0.5, 0.25, 2, xboxController, log));

    // Intake subsystem
    SmartDashboard.putData("Intake Front Fwd", new IntakeSetPercentOutput(0.2, 0.2, intakeFront, log));
    SmartDashboard.putData("Intake Front Rev", new IntakeSetPercentOutput(-0.2, -0.2, intakeFront, log));
    SmartDashboard.putData("Intake Front Stop", new IntakeStop(intakeFront, log));
    SmartDashboard.putData("Intake Front Deploy", new IntakePistonSetPosition(true, intakeFront, log));
    SmartDashboard.putData("Intake Front Retract", new IntakePistonSetPosition(false, intakeFront, log));

    // Shooter subsystem
    SmartDashboard.putData("Shooter Stop", new ShooterStop(shooter, log));
    SmartDashboard.putData("Shooter Set Percent", new ShooterSetPercentOutput(shooter, log));
    SmartDashboard.putData("Shooter Set PID", new ShooterSetPIDSV(shooter, log));
    SmartDashboard.putData("Shooter Set Velocity", new ShooterSetVelocity(InputMode.kSpeedRPM, shooter, log));
    SmartDashboard.putData("Shooter RPM from Distance", new ShooterSetVelocity(InputMode.kDistFeet, shooter, log));
    SmartDashboard.putData("Shooter Calibrate Fwd", new ShooterRampOutput(0, 0.9, 30.0, shooter, log));
    SmartDashboard.putData("Shooter Distance to RPM", new ShooterDistToRPM(shooter, log));
    SmartDashboard.putData("Shoot Ball RPM from Distance", new ShootBall(shooter, uptake, feeder, log));

    // Feeder subsystem
    SmartDashboard.putData("Set Feeder Percent", new FeederSetPercentOutput(feeder, log));
    SmartDashboard.putData("Stop Feeder", new FeederStop(feeder, log));

    // Feeder sequences
    SmartDashboard.putData("Start Feeder Sequence", new FeederBallReadyToShoot(feeder, log));
    SmartDashboard.putData("Shoot Red Ball Sequence", new FeedAndShootBall(shooter, feeder, uptake, log, BallColor.kBlue));
    SmartDashboard.putData("Shoot Blue Ball Sequence", new FeedAndShootBall(shooter, feeder, uptake, log, BallColor.kRed));

    // Uptake subsystem and sequences
    SmartDashboard.putData("Uptake and Eject Run Upward", new UptakeSetPercentOutput(.25, false, uptake, log));
    SmartDashboard.putData("Uptake Eject Ball", new UptakeSetPercentOutput(.25, true, uptake, log));
    SmartDashboard.putData("Uptake Only Run Upward", new UptakeSetPercentOutput(0.15, 0, uptake, log));
    SmartDashboard.putData("Uptake Stop", new UptakeStop(uptake, log));
    //SmartDashboard.putData("Uptake Reject Blue", new UptakeSortBall(BallColor.kBlue, uptake, feeder, log));
    //SmartDashboard.putData("Uptake Reject Red", new UptakeSortBall(BallColor.kRed, uptake, feeder, log));

    // Turret subsystem
    SmartDashboard.putData("Turret Stop", new TurretStop(turret, log));
    SmartDashboard.putData("Turret Set Percent", new TurretSetPercentOutput(turret, log));
    SmartDashboard.putData("Turret Calibrate Fwd", new TurretRampOutput(0, 0.3, 10.0, turret, log));  
    SmartDashboard.putData("Turret Turn to Angle", new TurretTurnAngle(TargetType.kAbsolute, false, turret, pivisionhub, log));
    SmartDashboard.putData("Turret Turn Vision Twice", new TurretTurnAngleTwice(TargetType.kVisionOnScreen, false, turret, pivisionhub, log));

    // Shooter camera subsystem
    SmartDashboard.putData("shooter-cam ToggleLED", new PiVisionHubSetLEDState(2, pivisionhub));
    SmartDashboard.putData("shooter-cam LEDOn", new PiVisionHubSetLEDState(1, pivisionhub));
    SmartDashboard.putData("shooter-cam LEDOff", new PiVisionHubSetLEDState(0, pivisionhub));

    // buttons for testing drive code, not updating numbers from SmartDashboard
    SmartDashboard.putData("DriveForward", new DriveSetPercentOutput(0.4, 0.4, driveTrain, log));
    SmartDashboard.putData("DriveBackward", new DriveSetPercentOutput(-0.4, -0.4, driveTrain, log));
    SmartDashboard.putData("DriveTurnLeft", new DriveSetPercentOutput(-0.4, 0.4, driveTrain, log));
    SmartDashboard.putData("DriveTurnRight", new DriveSetPercentOutput(0.4, -0.4, driveTrain, log));
    SmartDashboard.putData("DriveStraightRel", new DriveStraight(3, TargetType.kRelative, 0.0, 2.66, 3.8, true, driveTrain, limeLightFront, log));
    SmartDashboard.putData("DriveStraightAbs", new DriveStraight(3, TargetType.kAbsolute, 0.0, 2.66, 3.8, true, driveTrain, limeLightFront, log));
    SmartDashboard.putData("DriveStraightVis", new DriveStraight(3, TargetType.kVisionOnScreen, 0.0, 2.66, 3.8, true, driveTrain, limeLightFront, log));
    // SmartDashboard.putData("Drive Vision Assist", new VisionAssistSequence(driveTrain, limelightFront, log, shooter, feeder, led, hopper, intake));
    SmartDashboard.putData("TurnVision", new DriveTurnGyro(TargetType.kVisionOnScreen, 0, 90, 100, true, 2, driveTrain, limeLightFront, log));
    SmartDashboard.putData("TurnRelative", new DriveTurnGyro(TargetType.kRelative, 90, 90, 100, 1, driveTrain, limeLightFront, log));
    SmartDashboard.putData("TurnAbsolute", new DriveTurnGyro(TargetType.kAbsolute, 90, 90, 100, 1, driveTrain, limeLightFront, log));

    // DriveTrain calibration
    SmartDashboard.putData("DriveStraightManual", new DriveStraight(TargetType.kRelative, true, driveTrain, limeLightFront, log));
    SmartDashboard.putData("Drive Cal Slow", new DriveCalibrate(0.3, 35, 0.01, CalibrateMode.kStraight, driveTrain, log));
    SmartDashboard.putData("Drive Cal Fast", new DriveCalibrate(0.5, 12, 0.05, CalibrateMode.kStraight, driveTrain, log));

    SmartDashboard.putData("TurnGyroManual", new DriveTurnGyro(TargetType.kRelative, false, driveTrain, limeLightFront, log));
    SmartDashboard.putData("TurnCal Left Slow", new DriveCalibrate(0.3, 35, 0.01, CalibrateMode.kTurnLeft, driveTrain, log));
    SmartDashboard.putData("TurnCal Right Slow", new DriveCalibrate(0.3, 35, 0.01, CalibrateMode.kTurnRight, driveTrain, log));
    SmartDashboard.putData("TurnCal Left Fast", new DriveCalibrate(0.3, 10, 0.05, CalibrateMode.kTurnLeft, driveTrain, log));
    SmartDashboard.putData("TurnCal Right Fast", new DriveCalibrate(0.3, 10, 0.05, CalibrateMode.kTurnRight, driveTrain, log));
    SmartDashboard.putData("TurnCal Left Step0.2", new DriveCalibrate(0.2, 6, 3, CalibrateMode.kTurnLeft, driveTrain, log));
    SmartDashboard.putData("TurnCal Right Step0.2", new DriveCalibrate(0.2, 6, 3, CalibrateMode.kTurnRight, driveTrain, log));
    SmartDashboard.putData("TurnCal Left Step0.3", new DriveCalibrate(0.3, 6, 3, CalibrateMode.kTurnLeft, driveTrain, log));
    SmartDashboard.putData("TurnCal Right Step0.3", new DriveCalibrate(0.3, 6, 3, CalibrateMode.kTurnRight, driveTrain, log));
    SmartDashboard.putData("TurnCal Left-Right", new SequentialCommandGroup(
      new DriveCalibrate(0.3, 3, 15, CalibrateMode.kTurnLeft, driveTrain, log),
      new DriveCalibrate(0.3, 3, 15, CalibrateMode.kTurnRight, driveTrain, log)
    ) );

    // Testing for autos and trajectories
    SmartDashboard.putData("ZeroGyro", new DriveZeroGyro(driveTrain, log));
    SmartDashboard.putData("ZeroEncoders", new DriveZeroEncoders(driveTrain, log));
    SmartDashboard.putData("ZeroOdometry", new DriveResetPose(0, 0, 0, driveTrain, log));
    SmartDashboard.putData("Drive Trajectory Relative", new DriveFollowTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], false, PIDType.kTalon, driveTrain, log));
    SmartDashboard.putData("Drive Trajectory Curve Relative", new DriveFollowTrajectory(CoordType.kRelative, StopType.kBrake, trajectoryCache.cache[TrajectoryType.testCurve.value], false, PIDType.kTalon, driveTrain, log));
    SmartDashboard.putData("Drive Trajectory Absolute", new DriveFollowTrajectory(CoordType.kAbsolute, StopType.kBrake, trajectoryCache.cache[TrajectoryType.test.value], driveTrain, log));  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    JoystickButton[] xb = new JoystickButton[11];
    //check povtrigger and axis trigger number bindings
    Trigger xbPOVUp = new POVTrigger(xboxController, 0);
    Trigger xbPOVRight = new POVTrigger(xboxController, 90);
    //Trigger xbPOVDown = new POVTrigger(xboxController, 180);
    Trigger xbPOVLeft = new POVTrigger(xboxController, 270);
    
    Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
    Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);
    
    // right trigger shoots ball
    xbRT.onTrue(new ShootSequence(uptake, feeder, shooter, log).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // xbRT.whenActive(new ShootSequence(uptake, feeder, shooter, log),false);
    // xbRT.whenInactive(new ShootSequenceStop(uptake, feeder, log));

    // left trigger aim turret
    // xbLT.whenActive(new SequentialCommandGroup(
    //   new PiVisionHubSetLEDState(1, pivisionhub),
    //   new WaitCommand(0.07), // TODO change?
    //   new TurretTurnAngle(TargetType.kVisionOnScreen, 0, -1, turret, pivisionhub, log)
    // ));
    //xbLT.whenActive(new TurretTurnAngle(TargetType.kVisionOnScreen, 0, -1, turret, pivisionhub, log));
    xbLT.onTrue(new TurretTurnAngleTwice(TargetType.kVisionOnScreen, false, turret, pivisionhub, log));
    // xbLT.whenInactive(new ParallelCommandGroup(
    //   new TurretStop(turret, log),
    //   new PiVisionHubSetLEDState(0, pivisionhub) 
    // ));
    xbLT.onFalse(new TurretStop(turret, log));

    for (int i = 1; i < xb.length; i++) {
      xb[i] = new JoystickButton(xboxController, i);
    }
    
    //a - short shot distance
    xb[1].whileTrue(new ShootSetup(false, 3100, pivisionhub, shooter, log));         
    // xb[1].whenReleased(new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log));
    
    //b - medium shot distance
    xb[2].whileTrue(new ShootSetup(false, 3400, pivisionhub, shooter, log));        
    // xb[2].whenReleased(new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log));

    //y - long shot distance
    xb[4].whileTrue(new ShootSetup(false, 4100, pivisionhub, shooter, log));        
    // xb[4].whenReleased(new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log));
    
    //x - shot speed using vision
    xb[3].whileTrue(new ShootSetup(true, 3100, pivisionhub, shooter, log));        
    
    // LB = 5, RB = 6
    xb[5].onTrue(new TurretSetPercentOutput(-0.1, turret, log));
    xb[5].onFalse(new TurretStop(turret, log));
    xb[6].onTrue(new TurretSetPercentOutput(+0.1, turret, log));
    xb[6].onFalse(new TurretStop(turret, log));

    // back = 7, start = 8 
    xb[7].whileTrue(new ShootSetup(false, 500, pivisionhub, shooter, log));   // micro shot for use in the pit
    // xb[7].whenHeld(new ClimberSetExtended(true,climber, log)); 
    xb[8].whileTrue(new ClimberSetExtended(false,climber, log)); 

    // pov is the d-pad (up, down, left, right)
    xbPOVUp.onTrue(new TurretTurnAngle(TargetType.kAbsolute, 0, 2, turret, pivisionhub, log));
    xbPOVRight.onTrue(new TurretTurnAngle(TargetType.kAbsolute, 45, 2, turret, pivisionhub, log));
    // xbPOVRight.whenActive(new TurretTurnAngle(TargetType.kVisionScanRight, 45, -2, turret, pivisionhub, log));
    xbPOVLeft.onTrue(new TurretTurnAngle(TargetType.kAbsolute, -45, 2, turret, pivisionhub, log));
    // xbPOVLeft.whenActive(new TurretTurnAngle(TargetType.kVisionScanLeft, 45, -2, turret, pivisionhub, log));
    //xbPOVDown.whenActive(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // left joystick left button
    //left[1].whenPressed(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));

    //left[1].whenPressed(new IntakeToColorSensor(intakeFront, uptake, log));

    // left joystick right button
    //left[2].whenPressed(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log));

    // right joystick left button
    right[1].whenPressed(new IntakeExtendAndTurnOnMotors(intakeFront, uptake, log)); 

    // right joystick right button
    right[2].whenPressed(new IntakeRetractAndFlush(intakeFront, uptake, feeder, log)); 

    // right[2].whileHeld(new DriveTurnGyro(TargetType.kVision, 0, 90, 100, true, 1, driveTrain, limelightFront, log)); // turn gyro with vision
 
  }

  /** 
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    coP[1].whenPressed(new IntakeToColorSensor(intakeFront, uptake, log)); 
    coP[2].whenPressed(new ShooterSetVelocity(InputMode.kSpeedRPM, ShooterConstants.shooterDefaultRPM, shooter, log)); 

    coP[3].whenPressed(new UptakeFeedBall(uptake, feeder, log)); 
    coP[4].whenPressed(new UptakeEjectBall(uptake, log)); 

    // coP[5].whenHeld(new ClimbSetPercentOutput(0.4, climb, log)); // manually raise climb arms, slowly
    // coP[6].whenHeld(new ClimbSetPercentOutput(-0.4, climb, log)); // manually lower climb arms, slowly
    
    // top row RED SWITCH
    coP[8].whenPressed(new StopAllMotors(feeder, shooter, intakeFront, uptake, log));

    // middle row UP then DOWN, from LEFT to RIGHT
    coP[9].whenPressed(new IntakeSetPercentOutput(IntakeConstants.onPct, IntakeConstants.onPctTransfer, intakeFront, log)); // forward intake and transfer
    coP[10].whenPressed(new IntakeSetPercentOutput(-IntakeConstants.onPct, -IntakeConstants.onPctTransfer, intakeFront, log)); // reverse intake and transfer

    coP[11].whenPressed(new UptakeSetPercentOutput(-UptakeConstants.onPct, 0, uptake, log)); // reverse uptake
    coP[12].whenPressed(new UptakeSetPercentOutput(UptakeConstants.onPct, 0, uptake, log)); // forward uptake

    coP[13].whenPressed(new FeederSetPercentOutput(-FeederConstants.onPct, feeder, log)); // reverse feeder
    coP[14].whenPressed(new FeederSetPercentOutput(FeederConstants.onPct, feeder, log)); // forward feeder

    // middle row UP OR DOWN, fourth button
    coP[7].whenPressed(new IntakePistonToggle(intakeFront, uptake, log)); 

    // bottom row UP then DOWN, from LEFT to RIGHT
    coP[15].whenPressed(new ClimberSetExtended(true,climber, log)); // climb extend
    coP[16].whenPressed(new ClimberSetExtended(false,climber, log)); // climb retract
  }


  /**
   * Sets the rumble on the XBox controller
   * @param percentRumble The normalized value (0 to 1) to set the rumble to
   */
	public void setXBoxRumble(double percentRumble) {
		xboxController.setRumble(RumbleType.kLeftRumble, percentRumble);
    xboxController.setRumble(RumbleType.kRightRumble, percentRumble);

    if (percentRumble == 0) rumbling = false;
    else rumbling = true;
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(driveTrain, shooter, feeder, intakeFront, uptake, turret, pivisionhub, limeLightFront, log);
  }

  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }

    limeLightFront.setLedMode(1);     // Turn off LEDs on front limelight
    // limeLightRear.setLedMode(1);      // Turn off LEDs on rear limelight
    pivisionhub.setLEDState(false);
    // pivisionhub.setLEDState(true);

    //compressor.disable();
    compressor.enabled();
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    limeLightFront.setLedMode(1);     // Turn off LEDs on front limelight
    // limeLightRear.setLedMode(1);      // Turn off LEDs on rear limelight
    pivisionhub.setLEDState(false);

    driveTrain.setDriveModeCoast(true);     // When pushing a disabled robot by hand, it is a lot easier to push in Coast mode!!!!
  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    boolean error = true;
    if ((driveTrain.getLeftBusVoltage() > 8.0) && (driveTrain.isGyroReading() == true) && (driveTrain.getRightTemp() > 5.0)) error = false;    // The CAN bus and Gyro are working
  
    if (error) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }  //    TODO May want to flash this
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");

    driveTrain.setDriveModeCoast(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");

    pivisionhub.setLEDState(true);
    driveTrain.setDriveModeCoast(false);

    // wait until teleop to set trigger as it interferes with autos
    configureSensorTriggers();
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {
    // if (limelightFront.seesTarget()) {
    //   setXBoxRumble(1.0);
    // } else if (intake.intakeGetPercentOutput() == Math.abs(Constants.IntakeConstants.intakeDefaultPercentOutput)) {
    //   setXBoxRumble(0.4);
    // } else {
    //   setXBoxRumble(0);
    // }
  }
}
