/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkyHook;

public class SkyHook_MoveElevator extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_skyHook;
  private final double m_position;

  public SkyHook_MoveElevator(SkyHook _skyHook, Double _position) {
    m_skyHook = _skyHook;
    m_position = _position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_skyHook);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_position == 0){
      m_skyHook.SetExtensionPower(m_position);
      m_skyHook.setManualControlMode(false);
    }
    else {
      m_skyHook.setManualControlMode(true);
      double chk = SmartDashboard.getNumber("Extension To", 0);
      if (chk  != 0) { 
        m_skyHook.SetExtensionPosition(chk); 
      }
      else {
        m_skyHook.SetExtensionPosition(m_position);
      }
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
