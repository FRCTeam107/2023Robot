/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TRexArms;

public class DownTRexArms extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final TRexArms m_tRexArms;



 
  public DownTRexArms(TRexArms _tRexArms) {
    m_tRexArms = _tRexArms;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tRexArms);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tRexArms.runSlapper(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
