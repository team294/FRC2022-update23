/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.utilities.StringUtil.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallLocation;
import frc.robot.Constants.Ports;
import frc.robot.utilities.BallCount;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;



public class Feeder extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final WPI_TalonFX motor;

  private DigitalInput ballSensor = new DigitalInput(Ports.DIOFeederBallSensor) ;

  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private boolean fastLogging = false;  // true is enabled to run every cycle; false follows normal logging cycles

  public Feeder(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    motor = new WPI_TalonFX(Ports.CANFeeder);

    // set motor configuration
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.setNeutralMode(NeutralMode.Brake);
    // motor.setNeutralMode(NeutralMode.Coast);
    motor.configPeakOutputForward(1.0);
    motor.configPeakOutputReverse(-1.0);
    motor.configNeutralDeadband(0.01);
    motor.configVoltageCompSaturation(12);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.05);   //seconds from neutral to full
    motor.configClosedloopRamp(0.05); //seconds from neutral to full

  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * 
   * @return true = ball is in feeder
   */
  public boolean isBallPresent(){
    return !ballSensor.get();
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
   * @param voltage voltage
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * sets the percent of the motor
   * @param percent percent
   */
  public void setMotorPercentOutput(double percent){
    motor.set(ControlMode.PercentOutput, percent);
  }

  /**
  * Stops the motor
  */
  public void stopMotor(){
    setMotorPercentOutput(0);
  }


  @Override
  public void periodic(){
    
    if(fastLogging || log.getLogRotation() == log.FEEDER_CYCLE) {
      updateLog(false);
      SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), motor.getMotorOutputVoltage());
      SmartDashboard.putBoolean(buildString(subsystemName, " Is Ball Present"), isBallPresent());

      if (isBallPresent()) {
        BallCount.setBallCount(1, BallLocation.kFeeder, log);
      } else {
        BallCount.setBallCount(0, BallLocation.kFeeder, log);
      }      
     
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", motor.getBusVoltage(),
      "Out Percent", motor.getMotorOutputPercent(),
      "Volt", motor.getMotorOutputVoltage(), 
      "Amps", motor.getSupplyCurrent(),
      "Is Ball", isBallPresent()
    );
  }


}
