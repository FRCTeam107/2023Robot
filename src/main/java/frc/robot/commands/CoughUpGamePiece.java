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
import frc.robot.subsystems.PancakeFlipper;
import frc.robot.subsystems.TRexArms;
import frc.robot.subsystems.PancakeFlipper.FlipperPosition;
import frc.robot.subsystems.PancakeFlipper.IntakeConstants;

public class CoughUpGamePiece extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final TRexArms m_tRexArms;
  private final PancakeFlipper m_pancakeFlipper;



 
  public CoughUpGamePiece(TRexArms _tRexArms, PancakeFlipper _pancakeFlipper) {
    m_tRexArms = _tRexArms;
    m_pancakeFlipper = _pancakeFlipper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tRexArms, m_pancakeFlipper);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_tRexArms.runClapper(0, 0);
    m_pancakeFlipper.SetFlipperPos(FlipperPosition.PICKUP);
    m_pancakeFlipper.RunPickupMotors(IntakeConstants.ejectPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_pancakeFlipper.SetFlipperPos(FlipperConstants.homePos);
    m_pancakeFlipper.RunPickupMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
