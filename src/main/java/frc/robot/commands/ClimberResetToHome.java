/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkyHook;

public class ClimberResetToHome extends CommandBase {

  private final SkyHook m_climber;


  public ClimberResetToHome(SkyHook climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_climber.pullArmForward();
    m_climber.pullHook();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopHook();
    m_climber.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean armReady = m_climber.armHitForwardLimit();
    boolean hookReady = m_climber.hookHitForwardLimit();

    return (armReady && hookReady);
  }
}
