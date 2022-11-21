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
 * D-Pad on xBoxController.
 */
public class POVTrigger extends Trigger {
  private Joystick joystick;
  private int pov;
  private int direction;
  
  /**
   * Creates trigger that is active when given joystick
   * POV control is pressed in specified direction.
   * @param joystick joystick to monitor
   * @param pov joystick POV ID number to monitor
   * @param direction direction to trigger on, in degrees (up = 0, right = 90, down = 180, left = 270)
   */
  public POVTrigger(Joystick joystick, int pov, int direction) {
      this.joystick = joystick;
      this.pov = pov;
      this.direction = direction;
  }

  /**
   * Creates a trigger that is active when given joystick
   * POV control (ID = 0) is pressed in specified direction.  
   * @param joystick Joystick to monitor
   * @param direction Direction directionue to trigger on, in degrees (Up = 0, Right = 90, Down = 180, Left = 270)
   */
  public POVTrigger(Joystick joystick, int direction) {
      this.joystick = joystick;
      this.pov = 0;
      this.direction = direction;
  }

  /**
   * @return true if pov is triggered in specified direction
   */
  public boolean get() {
      return joystick.getPOV(pov) == direction;
  }
}