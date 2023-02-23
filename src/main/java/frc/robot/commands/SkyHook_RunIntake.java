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


public class SkyHook_RunIntake extends CommandBase {
  /**
   * Creates a new Shoot.`
   */
  private final SkyHook m_SkyHook;
  private final double m_setpoint;

  public SkyHook_RunIntake(SkyHook _SkyHookFlipper, Double _setpoint) {
    m_SkyHook = _SkyHookFlipper;
    m_setpoint = _setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_SkyHook);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_SkyHook.SetIntakePower(m_setpoint);
    // m_SkyHook.SetFlipperPos(m_position);
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
