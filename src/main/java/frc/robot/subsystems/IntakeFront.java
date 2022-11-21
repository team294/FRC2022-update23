// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import static frc.robot.utilities.StringUtil.*;

public class IntakeFront extends Intake {
  private final WPI_TalonFX transfer;

  /** Creates a new IntakeFront.  This is an intake with an extra "transfer" motor
   * between the intake and the uptake.
  */
  public IntakeFront(FileLog log) {
    super("Intake-Front", Ports.CANIntakeFront, Ports.SolIntakeFrontFwd, Ports.SolIntakeFrontRev, log);

    transfer = new WPI_TalonFX(Ports.CANIntakeFrontTransfer);

    // set transfer motor configuration
    transfer.configFactoryDefault();
    transfer.setInverted(false);
    transfer.setNeutralMode(NeutralMode.Brake);
    transfer.configPeakOutputForward(1.0);
    transfer.configPeakOutputReverse(-1.0);
    transfer.configNeutralDeadband(0.01);
    transfer.configVoltageCompSaturation(IntakeConstants.compensationVoltage);
    transfer.enableVoltageCompensation(true);
    transfer.configOpenloopRamp(0.05);   //seconds from neutral to full
  }

  /**
   * Sets the voltage of both the Intake motor and the transfer motor
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param voltage voltage (sets both motors to same voltage)   (+ = intake, - = outtake)
   */
  @Override
  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
    transfer.setVoltage(voltage);
  }

  /**
   * Sets the voltage of both the Intake motor and the transfer motor
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param intakeVoltage voltage for intake motor  (+ = intake, - = outtake)
   * @param transferVoltage voltage for transfer motor  (+ = intake, - = outtake)
   */
  public void setVoltage(double intakeVoltage, double transferVoltage) {
    super.setVoltage(intakeVoltage);
    transfer.setVoltage(transferVoltage);
  }
  
  /**
   * Sets the percent of the Intake motor and the transfer motor, using voltage compensation if turned on.
   * Sets both motors to same percent
   * @param percent -1 to +1 (+ = intake, - = outtake)
   */
  @Override
  public void setMotorPercentOutput(double percent){
    super.setMotorPercentOutput(percent);
    transfer.set(ControlMode.PercentOutput, percent);
  }

  /**
   * sets the percent of the Intake motor and the transfer motor, using voltage compensation if turned on
   * @param intakePercent percent for intake motor, -1 to +1  (+ = intake, - = outtake)
   * @param transferPercent percent for transfer motor, -1 to +1  (+ = intake, - = outtake)
   */
  public void setMotorPercentOutput(double intakePercent, double transferPercent){
    super.setMotorPercentOutput(intakePercent);
    transfer.set(ControlMode.PercentOutput, transferPercent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();

    if(fastLogging || log.getLogRotation() == log.INTAKE_CYCLE) {
      SmartDashboard.putNumber(buildString(getName(), " Transfer Voltage"), transfer.getMotorOutputVoltage());
      SmartDashboard.putNumber(buildString(getName(), " Transfer Temperature C"), transfer.getTemperature());
    }
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  @Override
	public void updateLog(boolean logWhenDisabled) {
		log.writeLog(logWhenDisabled, getName(), "Update Variables",  
      super.logString(),
      "Trans Out Percent", transfer.getMotorOutputPercent(),
      "Trans Volt", transfer.getMotorOutputVoltage(), 
      "Trans Amps", transfer.getSupplyCurrent(),
      "Trans Temperature", transfer.getTemperature()
    );
  }
}
