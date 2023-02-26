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

public class SkyHook_MoveWrist extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_SkyHook;
  private final double m_position;

  public SkyHook_MoveWrist(SkyHook _SkyHookFlipper, Double _position) {
    m_SkyHook = _SkyHookFlipper;
    m_position = _position;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SkyHook);
   
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
      m_SkyHook.SetWristPower(0.0);
    }
    else {
      //double val = SmartDashboard.getNumber("WristSetpoint",0.0);
      m_SkyHook.SetWristPosition(m_position);
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
