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
import frc.robot.subsystems.SkyHook.ArmPositions;


public class SkyHook_MoveArm extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_skyHook;
  private final double m_setPoint;
  private boolean m_incrementMove;

  public SkyHook_MoveArm(SkyHook _skyHook, Double _position, boolean _incrementalMovement) {
    m_skyHook = _skyHook;
    m_setPoint = _position;
    m_incrementMove = _incrementalMovement;

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
    else if (m_incrementMove){
      // increment or decrement position, but not checking limits here
      // allow the SkyHook subsystem to manage the limit checks
        double newPosition  = 0;//m_skyHook.GetArmHoldSetpoint() + m_setPoint;
        m_skyHook.SetArmPosition(newPosition);
        //m_skyHook.SetArmPower(m_setPoint);
    }
    else {
       m_skyHook.SetArmSmartMotion(m_setPoint);
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