// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class MathBCR {

/**
 * Converts input angle to a number between -179.999 and +180.0.
 * @return normalized angle
 */
public static double normalizeAngle(double angle) {
    angle = angle % 360;
    angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
    return angle;
  }
}
