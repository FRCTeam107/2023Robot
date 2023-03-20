/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkyHook;

public class SkyHook_MoveWrist extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_Wrist;
  private final double m_position;

  public SkyHook_MoveWrist(SkyHook _Wrist, Double _position) {
    m_Wrist = _Wrist;
    m_position = _position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
   
    //double val =  SmartDashboard.getNumber("WristSetpoint", 0.0);
    //SmartDashboard.putNumber("WristSetpoint", val);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_SkyHook.SetWristPower(m_position);
    if (m_position == 0){
      m_Wrist.SetWristPower(0.0);
      m_Wrist.setManualControlMode(false);
    }
    else {
      m_Wrist.setManualControlMode(true);
      double chk = SmartDashboard.getNumber("Wrist To", 0);
      if (chk  != 0) { 
        m_Wrist.SetWristPosition(chk); 
      }
      else {
        m_Wrist.SetWristPosition(m_position);
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
