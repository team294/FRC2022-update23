// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PiVisionHub;

public class PiVisionHubSetLEDState extends CommandBase {
  private PiVisionHub camera;
  private int state;

  /**
   * Changes PiVision LED State
   * @param val 0 for off, 1 for on, 2 for toggle
   * @param camera
   */
  public PiVisionHubSetLEDState(int state, PiVisionHub camera) {
    this.camera = camera;
    this.state = state;
    addRequirements(camera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (state) {
      case 0:
        camera.setLEDState(false);
        break;
      case 1:
        camera.setLEDState(true);
        break;
      case 2:
        camera.setLEDState(!camera.LEDState());
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
