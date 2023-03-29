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

public class SkyHook_ScoringPlus extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_skyHook;
  private final double m_armSetPoint;
  private final double m_extensionSetPoint;
  private final double m_wristSetPoint;
  private final Joystick m_Joystick;
  //private final double m_intakeSpeed;

  public SkyHook_ScoringPlus(SkyHook _skyHook, Double _armPosition, Double _extensionPosition, Double _wristPosition, Joystick _controller) {
    m_skyHook = _skyHook;
    m_armSetPoint = _armPosition;
    m_extensionSetPoint = _extensionPosition;
    m_wristSetPoint = _wristPosition;
    m_Joystick = _controller;
    //m_intakeSpeed = _intakeSpeed;


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
    m_skyHook.setManualControlMode(false);
    
    double adjustFactor = m_Joystick.getThrottle(); // + 1.0) / 2.0; // get number from -1 to 1
    SmartDashboard.putNumber("arm adjuster", m_Joystick.getThrottle());

    if (Math.abs(adjustFactor) > 0.1) {
      double adjustedSetPoint = m_armSetPoint;// - Math.copySign(adjustFactor * 200, adjustFactor);
      m_skyHook.SetArmSmartMotion(adjustedSetPoint);
    }
    else{
      m_skyHook.SetArmSmartMotion(m_armSetPoint);
    }
        
    m_skyHook.SetExtensionPosition(m_extensionSetPoint);
    m_skyHook.SetWristPosition(m_wristSetPoint);
    //m_skyHook.SetIntakePower(m_intakeSpeed);
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