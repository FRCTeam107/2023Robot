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
import frc.robot.subsystems.SkyHook.ExtensionPositions;
import frc.robot.subsystems.SkyHook.WristPositions;


public class SkyHook_ScoreBack extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_skyHook;
  private int ScoreTier = 0;

  public SkyHook_ScoreBack(SkyHook _skyHook) {
    m_skyHook = _skyHook;
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
    
    ScoreTier =(ScoreTier + 1) % 2;
    //SmartDashboard.putNumber("Score Back", ScoreTier);

    if ( ScoreTier == 1 ){
      m_skyHook.SetArmSmartMotion(ArmPositions.TIER2SCORE_BACK);
      m_skyHook.SetExtensionPosition(ExtensionPositions.TIER2SCORE_BACK);
      m_skyHook.SetWristPosition(WristPositions.TIER2SCORE_BACK);
    }
    else {
      m_skyHook.SetArmSmartMotion(ArmPositions.TIER3SCORE_BACK);
      m_skyHook.SetExtensionPosition(ExtensionPositions.TIER3SCORE_BACK);
      m_skyHook.SetWristPosition(WristPositions.TIER3SCORE_BACK);
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