/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkyHook;
import frc.robot.subsystems.SkyHook.ArmPositions;


public class SkyHook_MoveArmEnhanced extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_skyHook;
  private final double m_setPoint;
  private final Joystick m_Joystick;

  public SkyHook_MoveArmEnhanced(SkyHook _skyHook, Double _position, Joystick _controller) {
    m_skyHook = _skyHook;
    m_setPoint = _position;
    m_Joystick = _controller;

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
    if (m_setPoint == 0){
     m_skyHook.SetArmPower(m_setPoint);
     m_skyHook.setManualControlMode(false);
    }
    else {
      m_skyHook.setManualControlMode(true);
      double chk = SmartDashboard.getNumber("Arm To", 0);
      if (chk  != 0) { 
        m_skyHook.SetArmSmartMotion(chk); 
      }
      else { // moving by command
        double adjustFactor = m_Joystick.getThrottle(); // + 1.0) / 2.0; // get number from -1 to 1
        if (Math.abs(adjustFactor) > 0.1) {
          double adjustedSetPoint = m_setPoint + Math.copySign(adjustFactor * 10000, adjustFactor);
          m_skyHook.SetArmSmartMotion(adjustedSetPoint);
        }
        else {
          m_skyHook.SetArmSmartMotion(m_setPoint); 
        }
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