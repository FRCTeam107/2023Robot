/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDLights;

public class LED_ColorSet extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final LEDLights m_LEDS;
  private final String m_choice;

  public LED_ColorSet(LEDLights _LEDS, String _choice) {
    m_LEDS = _LEDS;
    m_choice = _choice;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDS);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_choice.toLowerCase() ==  "cone"){
      m_LEDS.lightsYellow();
    }
    else if (m_choice.toLowerCase() == "cube"){
        m_LEDS.lightsPurple();
    }
    else {
      m_LEDS.resetLights();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
