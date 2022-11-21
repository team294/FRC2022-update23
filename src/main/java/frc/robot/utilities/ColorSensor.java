// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import frc.robot.Constants.BallColor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
// import com.qualcomm.robotcore.hardware.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;

/**
 * Class used to read data from the RevRoboticsV3 Color Sensor.
 */
public class ColorSensor {
  /** Creates a new ColorSensor. */
	private final FileLog log;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kMXP);
  //private final ColorMatch colorMatcher = new ColorMatch();

  
  public ColorSensor(FileLog log) {
    this.log = log;

    SmartDashboard.putString("Last Color", BallColor.kNone.name());

     // colorMatcher.addColorMatch(kBlueTarget);
      //colorMatcher.addColorMatch(kGreenTarget);
      //colorMatcher.addColorMatch(kRedTarget);
  }

   /**
   * Get the raw proximity value from the color sensor ADC (11 bit). This value is largest when an object
   * is close to the sensor and smallest when far away.
   *
   * @return Proximity measurement value, ranging from 0 to 2047
   */
  public int getProximity() {
    return colorSensor.getProximity();
  }

  /**
   * Returns if there is a ball in the uptake.
   * @return true = ball is in the uptake.
   */
  public boolean isBallPresent() {
    return getProximity() >= 180;
  }

  /**
   * Returns the color detected in the uptake.
   * use as getColorNumber().red to get numerical r value in rgb
   * use as getColorNumber().green to get numerical g value in rgb
   * use as getColorNumber().blue to get numerical b value in rgb
   * @return Color object with the current detected color 
   */
  public Color getColor() {
    return colorSensor.getColor();
  }

  /**
   * Returns the nearest color for the ball.
   * Returns kNone if no ball is in the uptake.
   * Returns kOther if the ball is not red, blue, or yellow.
   * @return BallColor kNone, kRed, kBlue, kYellow, or kOther.
   */
  public BallColor getBallColor() {
    // Some color data (R,G,B,Prox) from prototype uptake, 2/4/22, proto board:
    // No ball (wood) = (0.31, 0.48, 0.21, 228)
    // Ball blue = (0.16, 0.41, 0.43, 690)
    // Ball red = (0.50, 0.36, 0.14, 700)
    // Ball yellow = (0.28, 0.60, 0.12, 1050)
    // Some color data (R,G,B,Prox) from prototype uptake, 2/4/22:
    // No ball (wood) = (0.28, 0.48, 0.24, 75)
    // Ball blue = (0.17, 0.41, 0.42, 288)
    // Ball red = (0.46, 0.37, 0.17, 300)
    // Ball yellow = (0.27, 0.60, 0.13, 312)
    Color color = getColor();

    if (!isBallPresent()) {
      return BallColor.kNone;
    } else if (color.blue >= 0.3 && color.red < 0.35 && color.green < 0.5) {
      return BallColor.kBlue;
    } else if (color.red > 0.35 && color.blue <= 0.3 && color.green < 0.5) {
      return BallColor.kRed;
    } else if (color.green > 0.5 && color.blue < 0.3) {
      return BallColor.kYellow;
    } 
    return BallColor.kOther;
  }

  /**
   * Returns the nearest color for the ball as a String.
   * Returns kNone if no ball is in the uptake.
   * Returns kOther if the ball is not red, blue, or yellow.
   * @return String = kNone, kRed, kBlue, kYellow, or kOther.
   */
  public String getBallColorString() {
    return getBallColor().name();
  }

  /**
   * @param check what color (based on target colors) to check for
   * @return whether or not the color "check" is being read
   
  public boolean isColor(Color check) {
      Color detectedColor = getColorNumber();

      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
      
      if (match.color == check) return true;
      return false;
  }
  **/

  /**
   * @return IR value being read by the IR sensor
   
  public double getIR() {
      double IR = colorSensor.getIR();
      SmartDashboard.putNumber("IR", IR);
      return colorSensor.getIR();
  }
  */
  
  public void updateShuffleboard() {
    Color color = getColor();

    SmartDashboard.putNumber("Color Proximity", getProximity());
    SmartDashboard.putNumber("Color Red", color.red);
    SmartDashboard.putNumber("Color Green", color.green);
    SmartDashboard.putNumber("Color Blue", color.blue);
    SmartDashboard.putString("Color", getBallColorString());
    if(isBallPresent()){
      SmartDashboard.putString("Last Color", getBallColorString());
    }
  }

  /**
   * Write information about color sensor to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
    Color color = getColor();

		log.writeLog(logWhenDisabled, "Color Sensor", "Update Variables",  
      "Color Recognized", getBallColorString(),
      "Color Proximity", getProximity(),
      "Color Red", color.red,
      "Color Green", color.green,
      "Color Blue", color.blue
    );
  }
}
