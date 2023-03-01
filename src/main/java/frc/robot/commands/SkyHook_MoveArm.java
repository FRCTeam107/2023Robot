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


public class SkyHook_MoveArm extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_skyHook;
  private final double m_setPoint;
  private boolean m_percentPower;

  public SkyHook_MoveArm(SkyHook _skyHook, Double _position, boolean _incrementalMovement) {
    m_skyHook = _skyHook;
    m_setPoint = _position;
    m_percentPower = _incrementalMovement;

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
     //m_skyHook.SetArmVelocity(m_setPoint);
    }
    else if (m_percentPower){
      double currPos  = m_skyHook.GetArmPosition();
      if (currPos < -10) {currPos = -10;}
      if (currPos > 10) {currPos = 10;}      
      
        double newPosition  = currPos + m_setPoint;
        if (newPosition < -10) {newPosition = -10;}
        if (newPosition > 10) {newPosition = 10;}

        m_skyHook.SetArmPosition(newPosition);
        //m_skyHook.SetArmPower(m_setPoint);
    }
    else {
      m_skyHook.SetArmPosition(m_setPoint);
      //m_skyHook.SetArmSmartMotion(m_position);
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
