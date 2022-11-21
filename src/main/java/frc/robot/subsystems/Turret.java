/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;

import static frc.robot.Constants.*;


public class Turret extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final WPI_TalonFX motor;

  private DigitalInput calSwitch = new DigitalInput(Ports.DIOTurretCalSwitch);    // Calibration switch on turret at start of match

  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private boolean encoderCalibrated = false;
  private double encoderZero = 0.0;     // Zero position for encoder, in raw units

  private int timeoutMs = 0; // was 30, changed to 0 for testing
  private double measuredDPS = 0.0;             // Current measured speed in turrent degrees per second

  
  public Turret(FileLog log) {
    this.log = log; // save reference to the fileLog
    subsystemName = "Turret";
    motor = new WPI_TalonFX(Ports.CANTurret);

    // set motor configuration
    motor.configFactoryDefault();
    motor.setInverted(true);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configNeutralDeadband(0.01);
    motor.configVoltageCompSaturation(TurretConstants.compensationVoltage);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.05);   //seconds from neutral to full
    motor.configClosedloopRamp(0.05); //seconds from neutral to full

    // set sensor configuration
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    motor.setSensorPhase(false);
    setEncoderCalibrated(false, 0.0);
    resetEncoderPosition(0.0);

    // set limit switch configuration
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
      LimitSwitchNormal.NormallyClosed, timeoutMs);
    motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
      LimitSwitchNormal.NormallyClosed, timeoutMs);
    motor.overrideLimitSwitchesEnable(true);
    // motor.configForwardSoftLimitThreshold(forwardSensorLimit, timeoutMs);
    // motor.configForwardSoftLimitEnable(true, timeoutMs);

    stopMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets the voltage of the motor
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param voltage voltage, + = turn to the right
   */
  public void setVoltage(double voltage) {
    // Soft limit for turret turning.
    if (encoderCalibrated) {
      if ( ( (getTurretPosition()>=TurretConstants.softLimitFwd) && (voltage>0) ) ||
           ( (getTurretPosition()<=TurretConstants.softLimitRev) && (voltage<0) ) ) {
        voltage = 0;
        log.writeLog(false, "Turret", "Set voltage - Soft limit stop", "Position", getTurretPosition(), "Fwd Limt", isAtFwdLimit(), "Rev Limit", isAtRevLimit());
      }
    }

    motor.setVoltage(voltage);
  }

  /**
   * sets the percent speed of the turret motor, using voltage compensation if turned on
   * @param percent percent -1.0 to +1.0, + = turn to the right
   */
  public void setPercentOutput(double percent){
    // Soft limit for turret turning.
    if (encoderCalibrated) {
      if ( ( (getTurretPosition()>=TurretConstants.softLimitFwd) && (percent>0) ) ||
           ( (getTurretPosition()<=TurretConstants.softLimitRev) && (percent<0) ) ) {
        percent = 0;
        log.writeLog(false, "Turret", "Set voltage - Soft limit stop", "Position", getTurretPosition(), "Fwd Limt", isAtFwdLimit(), "Rev Limit", isAtRevLimit());
      }
    }

    motor.set(ControlMode.PercentOutput, percent);
  }

  /**
  * Stops the motor
  */
  public void stopMotor(){
    setPercentOutput(0);
  }

  /**
   * Returns the state of the forward limit switch
   * @return true = turret at limit, false = not at limit
   */
  public boolean isAtFwdLimit() {
    return motor.isFwdLimitSwitchClosed()==0;
  }

  /**
   * Returns the state of the reverse limit switch
   * @return true = turret at limit, false = not at limit
   */
  public boolean isAtRevLimit() {
    return motor.isRevLimitSwitchClosed()==0;
  }

  /**
   * Returns the state of the calibration limit switch
   * @return true = turret at calibration location, false = not at limit
   */
  public boolean isAtCalSwitch() {
    return calSwitch.get();
  }


  /**
   * Returns the motor position
   * @return position of motor in raw units, without software zeroing
   */
  public double getMotorPositionRaw(){
    return motor.getSelectedSensorPosition(0);
  }

  /**
   * Returns the turret position
   * @return position of turret in degrees, + = to the right
   */
  public double getTurretPosition(){
    return (getMotorPositionRaw() - encoderZero)/TurretConstants.ticksPerDegree;
  }

  /**
	 * Zero the encoder position in software.
	 */
  public void zeroEncoder() {
    encoderZero = getMotorPositionRaw();
  }

  /**
	 * Resets the current encoder position in software.
   * @param currentPosition current turrent position (reset to this value), in degrees
   */
  public void resetEncoderPosition(double currentPosition) {
    encoderZero = getMotorPositionRaw() - currentPosition*TurretConstants.ticksPerDegree;
  }

  /**
   * @return velocity of turret in degrees per second, + = to the right
   */
  public double getTurretVelocity(){
    return motor.getSelectedSensorVelocity(0)*TurretConstants.rawVelocityToDegPerSec;
  }

  /**
   * Returns whether or not the encoder is calibrated to the correct turret angle
   * @return true = calibrated, false = uncalibrated
   */
  public boolean isEncoderCalibrated() {
    return encoderCalibrated;
  }

  /**
   * Sets the turret encoder as either calibrated or uncalibrated
   * @param calibrated true = calibrated, false = uncalibrated
   * @param currentPosition current turret position, in degrees.  Ignored if calibrated = false 
   */
  public void setEncoderCalibrated(boolean calibrated, double currentPosition) {
    encoderCalibrated = calibrated;
    SmartDashboard.putBoolean("Turret isCalibrated", encoderCalibrated);

    if (calibrated) {
      // Encoder is calibrated
      log.writeLog(false, "Turret", "setEncoderCalibrated", "Position", currentPosition);
      motor.configPeakOutputForward(TurretConstants.maxOutputCalibrated);
      motor.configPeakOutputReverse(-TurretConstants.maxOutputCalibrated);
      resetEncoderPosition(currentPosition);
    } else {
      // Encoder is no longer calibrated
      log.writeLog(false, "Turret", "setEncoderCalibrated", "Uncalibrated");
      motor.configPeakOutputForward(TurretConstants.maxOutputUncalibrated);
      motor.configPeakOutputReverse(-TurretConstants.maxOutputUncalibrated);
    }

  }


  @Override
  public void periodic(){
    measuredDPS = getTurretVelocity();

    // Auto-calibrate at limit switches
    if (!encoderCalibrated && isAtCalSwitch()) {
      setEncoderCalibrated(true, TurretConstants.calSwitch);
    } else if (isAtFwdLimit()) {
      setEncoderCalibrated(true, TurretConstants.limitSwitchFwd);
    } else if (isAtRevLimit()) {
      setEncoderCalibrated(true, TurretConstants.limitSwitchRev);      
    }

    // Safety stop at turret limit switches. Hardware should take care of this, but this is a double-check.
    if ( ( isAtFwdLimit() && (motor.getMotorOutputPercent()>0) ) ||
         ( isAtRevLimit() && (motor.getMotorOutputPercent()<0) ) ) {
      motor.stopMotor();
      log.writeLog(false, "Turret", "Hard limit stop", "Fwd Limt", isAtFwdLimit(), "Rev Limit", isAtRevLimit());
    }

    // Soft limit for turret turning.
    if (encoderCalibrated) {
      if ( ( (getTurretPosition()>=TurretConstants.softLimitFwd) && (motor.getMotorOutputPercent()>0) ) ||
           ( (getTurretPosition()<=TurretConstants.softLimitRev) && (motor.getMotorOutputPercent()<0) ) ) {
        motor.stopMotor();
        log.writeLog(false, "Turret", "Soft limit stop", "Position", getTurretPosition(), "Fwd Limt", isAtFwdLimit(), "Rev Limit", isAtRevLimit());
      }
    }

    if (fastLogging || log.getLogRotation() == log.TURRET_CYCLE) {
      updateLog(false);

      SmartDashboard.putNumber("Turret Voltage", motor.getMotorOutputVoltage());
      SmartDashboard.putNumber("Turret Position Deg", getTurretPosition());
      SmartDashboard.putNumber("Turret Velocity DegPerSec", measuredDPS);
      SmartDashboard.putNumber("Turret Temperature C", motor.getTemperature());
      SmartDashboard.putBoolean("Turret Fwd Limit", isAtFwdLimit());
      SmartDashboard.putBoolean("Turret Rev Limit", isAtRevLimit());
      SmartDashboard.putBoolean("Turret Rev Limit", isAtRevLimit());
      SmartDashboard.putBoolean("Turret Cal Switch", isAtCalSwitch());
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about turret to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", motor.getBusVoltage(),
      "Out Percent", motor.getMotorOutputPercent(),
      "Volt", motor.getMotorOutputVoltage(), 
      "Amps", motor.getSupplyCurrent(),
      "Temperature", motor.getTemperature(),
      "Position", getTurretPosition(),
      "Measured DegPerSec", measuredDPS,
      "Fwd Limit", isAtFwdLimit(),
      "Rev Limit", isAtRevLimit(),
      "Cal Switch", isAtCalSwitch()
    );
  }

  
}
