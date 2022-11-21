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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;
import static frc.robot.Constants.*;


public class Shooter extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final WPI_TalonFX motor1, motor2;
  
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private double encoderZero = 0.0;     // Zero position for encoder
  private int timeoutMs = 0; // was 30, changed to 0 for testing

  // Velocity control variables
  private double kS, kV;      // Feed forward parameters
  private boolean velocityControlOn = false;    // Is this subsystem using velocity control currently?
  private double measuredRPM = 0.0;             // Current measured speed
  private double setpointRPM = 0.0;             // Current velocity setpoint

  
  public Shooter(FileLog log) {
    this.log = log; // save reference to the fileLog
    subsystemName = "Shooter";
    motor1 = new WPI_TalonFX(Ports.CANShooter1);
    motor2 = new WPI_TalonFX(Ports.CANShooter2);

    // set motor1 configuration
    motor1.configFactoryDefault();
    motor1.setNeutralMode(NeutralMode.Coast);
    motor1.configPeakOutputForward(1.0);
    motor1.configPeakOutputReverse(-1.0);
    motor1.configNeutralDeadband(0.01);
    motor1.configVoltageCompSaturation(ShooterConstants.compensationVoltage);
    motor1.enableVoltageCompensation(true);
    motor1.configOpenloopRamp(0.15);   //seconds from neutral to full
    motor1.configClosedloopRamp(0.15); //seconds from neutral to full

    // set motor2 configuration
    motor2.configFactoryDefault();
    motor2.setNeutralMode(NeutralMode.Coast);
    motor2.configPeakOutputForward(1.0);
    motor2.configPeakOutputReverse(-1.0);
    motor2.configNeutralDeadband(0.01);
    motor2.configVoltageCompSaturation(ShooterConstants.compensationVoltage);
    motor2.enableVoltageCompensation(true);
    motor2.configOpenloopRamp(0.05);   //seconds from neutral to full
    motor2.configClosedloopRamp(0.05); //seconds from neutral to full

    // set motor2 to follow motor1
    motor1.setInverted(true);
    motor2.setInverted(false);
    motor2.set(ControlMode.Follower, Ports.CANShooter1);
    motor2.follow(motor1);

    // set sensor configuration
    motor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    motor1.setSensorPhase(false);

    zeroEncoder();

    // PID coefficients initial
    setPIDSV(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kI, 
      ShooterConstants.kS, ShooterConstants.kV);

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
   * @param voltage voltage, + = forward (shoot out)
   */
  public void setVoltage(double voltage) {
    log.writeLog(false,getName(), "SetVoltage", "Voltage", voltage);
    motor1.setVoltage(voltage);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
   * sets the percent of the motor, using voltage compensation if turned on
   * @param percent percent, + = forward (shoot out)
   */
  public void setMotorPercentOutput(double percent){
    log.writeLog(false,getName(), "SetPercentOutput", "Percent", percent);
    motor1.set(ControlMode.PercentOutput, percent);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
  * Stops the motor
  */
  public void stopMotor(){
    log.writeLog(false,getName(), "StopMotor");
    setMotorPercentOutput(0);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }


  /**
   * Returns the motor position
   * @return position of motor in raw units, without software zeroing
   */
  public double getMotorPositionRaw(){
    return motor1.getSelectedSensorPosition(0);
  }

  /**
   * Returns the motor position
   * @return position of motor in revolutions
   */
  public double getMotorPosition(){
    return (getMotorPositionRaw() - encoderZero)/ShooterConstants.ticksPerRevolution;
  }

  /**
	 * Zero the encoder position in software.
	 */
  public void zeroEncoder() {
    encoderZero = getMotorPositionRaw();
  }

  /**
   * @return velocity of motor in rpm
   */
  public double getMotorVelocity(){
    return motor1.getSelectedSensorVelocity(0)*ShooterConstants.rawVelocityToRPM;
  }


  /**
   * Sets PIDSV parameters for motor velocity control.
   * 
   * <p> For Feedback terms (P, I, D), the velocity error term is in units [encoder ticks]/[100ms].  The output drive is on a 1023 scale.
   * Note that RPM / ShooterConstants.rawVelocityToRPM will convert to [encoder ticks]/[100ms].
   * 
   * <p> For Feedforward terms, the velocity is in RPM.  The output drive is on a 1.0 scale.
   * @param P Proportional feedback term, in [1023-scale-output]/[ [encoder ticks]/[100ms] ]
   * @param I Integral feedback term.  Use a small IZone and start the I-gain at something small (0.001).
   * @param D Derivative feedback term.  D-gain can start typically at 10 X P-gain.
   * @param S Feedforward offset, in percent output (-1 to +1)
   * @param V Feedforward velocity term, in [percent output (-1 to +1)]/[RPM]
   */
  public void setPIDSV(double P, double I, double D, double S, double V)  {
    // set PID coefficients
    motor1.config_kP(0, P, timeoutMs);
    motor1.config_kI(0, I, timeoutMs);
    motor1.config_kD(0, D, timeoutMs);
    // motor.config_kF(0, F, timeoutMs);  // value = 1023 * desired-percent-out / at-sensor-velocity-sensor-units-per-100ms
    kS = S;
    kV = V;

    // auto clear the integral accumulator if the sensor value is too far from the
    // target. This prevent unstable oscillation if the kI is too large. Value is in sensor units [encoder ticks]/[100ms].
    motor1.config_IntegralZone(0, 100/ShooterConstants.rawVelocityToRPM, timeoutMs);

    // Sets the maximum integral accumulator, in closed loop error units [encoder ticks]/[100ms].
    motor1.configMaxIntegralAccumulator(0, 300/ShooterConstants.rawVelocityToRPM, timeoutMs);

    if (velocityControlOn) {
      // Reset velocity to force kS and kV updates to take effect
      setMotorVelocity(setpointRPM);
    }
  }

  /**
   * Run motor in a velocity closed loop mode.
   * @param motorRPM setPoint RPM, + = forward (shoot out)
   */
  public void setMotorVelocity(double motorRPM) {
    velocityControlOn = true;
    setpointRPM = motorRPM;
    
    motor1.set(ControlMode.Velocity, setpointRPM/ShooterConstants.rawVelocityToRPM,
      DemandType.ArbitraryFeedForward, kS*Math.signum(setpointRPM) + kV*setpointRPM);
  }

  /**
   * Returns that last rpm that was set
   */
  public double getSetpointRPM() {
    return setpointRPM;
  }
  
  /**
   * @return PID error, in RPM.  + = actual speed too fast, - = actual speed too slow
   */
  public double getVelocityPIDError() {
    return measuredRPM - setpointRPM;
  }

  /**
   * Given a distance from the target, return the RPM that the shooter should be at
   * 
   * @param distance distance from the target to the shooter
   * @return The RPM of the shooter motor.
   */
  public double distanceFromTargetToRPM(double distance) {
    // return 12.5*distance + 2050;
    // double rpm = 11.1243 * distance + 2488.02;
    double rpm = 13.3 * distance + 2394;    // Increase 12.5 to 13.0*D + 2400 for F1, F3: 15.9*d + 2267, F4:  14.6*d + 2300, F5: 13.3*d + 2394
    log.writeLog(false, "Shooter", "DistanceToRPM", "Distance", distance, "RPM", rpm);
    return rpm;
    // int len = ShooterConstants.distanceFromTargetToRPMTable.length;
    // if(distance < ShooterConstants.distanceFromTargetToRPMTable[0][0]) return ShooterConstants.shooterDefaultRPM; /*return ShooterConstants.distanceFromTargetToRPMTable[0][1];*/
    // if(distance > ShooterConstants.distanceFromTargetToRPMTable[len-1][0]) return ShooterConstants.distanceFromTargetToRPMTable[len-1][1];
    // int leftBound = 0;

    // for(int i = len - 1; i >= 0; i--) {
    //   if(distance > ShooterConstants.distanceFromTargetToRPMTable[i][0]) {
    //     leftBound = i;
    //     i = 0;
    //   } else if (distance == ShooterConstants.distanceFromTargetToRPMTable[i][0]) {
    //     return ShooterConstants.distanceFromTargetToRPMTable[i][1];
    //   }
    // }

    // double lowerRPM = ShooterConstants.distanceFromTargetToRPMTable[leftBound][1];
    // double upperRPM = ShooterConstants.distanceFromTargetToRPMTable[leftBound + 1][1];
    // double lowerDist = ShooterConstants.distanceFromTargetToRPMTable[leftBound][0];
    // double upperDist = ShooterConstants.distanceFromTargetToRPMTable[leftBound + 1][0];
    // double dRPMperFoot = (upperRPM - lowerRPM) / (upperDist - lowerDist);
    // double targetRPM = ((distance - lowerDist) * (dRPMperFoot)) + lowerRPM;
    // return targetRPM;
  }


  @Override
  public void periodic(){
    measuredRPM = getMotorVelocity();
    if(fastLogging || log.getLogRotation() == log.SHOOTER_CYCLE) {
      updateLog(false);

      SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), motor1.getMotorOutputVoltage());
      SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getMotorPosition());
      SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), measuredRPM);
      SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), motor1.getTemperature());
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
      "Bus Volt", motor1.getBusVoltage(),
      "Out Percent", motor1.getMotorOutputPercent(),
      "Out Percent 2", motor2.getMotorOutputPercent(),
      "Volt", motor1.getMotorOutputVoltage(), 
      "Volt 2", motor2.getMotorOutputVoltage(), 
      "Amps", motor1.getSupplyCurrent(),
      "Amps 2", motor2.getSupplyCurrent(),
      "Temperature", motor1.getTemperature(),
      "Temperature 2", motor2.getTemperature(),
      "Position", getMotorPosition(),
      "Measured RPM", measuredRPM,
      "Setpoint RPM", setpointRPM
    );
  }

  
}
