/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.triggers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * LT and RT on xBoxController.
 */
public class AxisTrigger extends Trigger {
  private Joystick joystick;
  private int axis;
  private double min;
  
  /**
   * Creates trigger that is active when given joystick
   * axis' value is gerater than min.
   * @param joystick joystick to monitor
   * @param axis joystick axis number to montior
   * @param min triggering minimum threshold
   */
  public AxisTrigger(Joystick joystick, int axis, double min) {
      this.joystick = joystick;
      this.axis = axis;
      this.min = min;
  }
  
  /**
   * @return true if axis value is greater than min
   */
  @Override
  public boolean getAsBoolean() {
      return joystick.getRawAxis(axis) > min;
  }
}