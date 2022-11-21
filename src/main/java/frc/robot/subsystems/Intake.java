/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;
import static frc.robot.Constants.*;


public class Intake extends SubsystemBase implements Loggable {
  protected final FileLog log;
  private final WPI_TalonFX motor;
  private final DoubleSolenoid intakePiston;
  
  protected boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private double encoderZero = 0.0;     // Zero position for encoder
  private boolean pistonExtended = false;     // Local copy of piston state to avoid CANbus delays when reading piston state right after setting it
  private int timeoutMs = 0; // was 30, changed to 0 for testing

  
  /**
   * Creates a generic intake (front or rear)
   * @param subsystemName  String name for subsystem
   * @param CANMotorPort
   * @param solenoidForwardChannel
   * @param solenoidReverseChannel
   * @param log
   */
  public Intake(String subsystemName, int CANMotorPort, int solenoidForwardChannel, int solenoidReverseChannel, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    motor = new WPI_TalonFX(CANMotorPort);
    intakePiston = new DoubleSolenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, solenoidForwardChannel, solenoidReverseChannel);

    // set Intake configuration
    motor.configFactoryDefault();
    motor.setInverted(true);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configPeakOutputForward(1.0);
    motor.configPeakOutputReverse(-1.0);
    motor.configNeutralDeadband(0.01);
    motor.configVoltageCompSaturation(IntakeConstants.compensationVoltage);
    motor.enableVoltageCompensation(true);
    motor.configOpenloopRamp(0.4);   //seconds from neutral to full.  Was 0.05, increased (G2) due to stripped belt on intake pulley
    motor.configClosedloopRamp(0.4); //seconds from neutral to full.  Was 0.05, increased (G2) due to stripped belt on intake pulley

    // set sensor configuration
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    motor.setSensorPhase(false);

    zeroEncoder();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets the voltage of the Intake
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param voltage voltage (+ = intake, - = outtake)
   */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * sets the percent of the Intake, using voltage compensation if turned on
   * @param percent -1 to 1 (+ = intake, - = outtake)
   */
  public void setMotorPercentOutput(double percent){
    motor.set(ControlMode.PercentOutput, percent);
  }

  /**
  * Stops the Intake
  */
  public void stopMotor(){
    setMotorPercentOutput(0);
  }


  /**
   * Returns the Intake position
   * 
   * @return position of Intake in raw units, without software zeroing
   */
  public double getMotorPositionRaw(){
    return motor.getSelectedSensorPosition(0);
  }

  /**
   * Returns the Intake position
   * @return position of Intake in revolutions
   */
  public double getMotorPosition(){
    return (getMotorPositionRaw() - encoderZero)/IntakeConstants.ticksPerRevolution;
  }

  /**
	 * Zero the encoder position in software.
	 */
  public void zeroEncoder() {
    encoderZero = getMotorPositionRaw();
  }

  /**
   * @return velocity of Intake in rpm
   */
  public double getMotorVelocity(){
    return motor.getSelectedSensorVelocity(0)*IntakeConstants.rawVelocityToRPM;
  }

  /**
   * Sets if the piston should be extended or not
   * 
   * @param extend true = extend, false = retract
   */
  public void setPistonExtended(boolean extend) {
    pistonExtended = extend;
    intakePiston.set(extend ? Value.kForward : Value.kReverse);
  }

  /**
   * Returns if intake piston is extended or not
   * @return true = extended, false = retracted
   */
  public boolean getPistonExtended() {
    return pistonExtended;
  }

  /**
   * Toggles the piston between retracted and deployed
   */
  public void togglePistonExtended(){
    log.writeLog(false, subsystemName, "togglePiston", "from extended", getPistonExtended());
    setPistonExtended(!getPistonExtended());
  }


  @Override
  public void periodic(){
    if(fastLogging || log.getLogRotation() == log.INTAKE_CYCLE) {
      updateLog(false);

      SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), motor.getMotorOutputVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getMotorPosition());
      SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), getMotorVelocity());
      SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), motor.getTemperature());
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
      logString()
    );
  }

  /**
   * Builds string for use in FileLog.  Useful when subclassing this Subsystem.
   * @return
   */
  protected String logString() {
    return buildStringWithCommas(
      "Bus Volt", motor.getBusVoltage(),
      "Out Percent", motor.getMotorOutputPercent(),
      "Volt", motor.getMotorOutputVoltage(), 
      "Amps", motor.getSupplyCurrent(),
      "Temperature", motor.getTemperature(),
      "Position", getMotorPosition(),
      "Measured RPM", getMotorVelocity(),
      "Piston extended", getPistonExtended()
    );
  }
}
