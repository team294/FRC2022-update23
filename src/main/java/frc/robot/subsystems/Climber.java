package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.Constants.*;


public class Climber extends SubsystemBase implements Loggable {
  protected final FileLog log;
  private final Solenoid leftPiston;
  private final Solenoid rightPiston;
  
  protected boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  
  /**
   * Creates a generic intake (front or rear)
   * @param subsystemName  String name for subsystem
   * @param CANMotorPort
   * @param solenoidChannel
   * @param log
   */
  public Climber(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    leftPiston = new Solenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolClimbLeft);
    rightPiston = new Solenoid(Ports.CANPneumaticHub, PneumaticsModuleType.REVPH, Ports.SolClimbRight);
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets if the piston should be extended or not
   * 
   * @param value true = extend, false = retract
   */
  public void setPistonExtended(boolean value) {
    leftPiston.set(value);
    rightPiston.set(value);
  }

  /**
   * Returns if intake piston is extended or not
   * @return true = extended, false = retracted
   */
  public boolean getPistonExtended() {
    if (leftPiston.get() == true) return true;
    else return false;
  }

  /**
   * Toggles the piston between retracted and deployed
   */
  public void togglePistonExtended(){
    log.writeLog(false, subsystemName, "togglePiston", "from extended", getPistonExtended());
    setPistonExtended(!getPistonExtended());
  }

  @Override
  public void periodic(){}

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

}
