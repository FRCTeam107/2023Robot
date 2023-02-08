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
import frc.robot.subsystems.SkyHook.ClimberConstants;

public class DismountFirstBar extends CommandBase {
    private final SkyHook m_climber;

    private enum commandState {
      Starting,
      PullHook,
      MoveArmForward,
      RaiseHook,
      Finished;

      public commandState getNext() {
        return this.ordinal() < commandState.values().length - 1
            ? commandState.values()[this.ordinal() + 1]
            : null;
      }
    }
    private static commandState currentState;



  public DismountFirstBar(SkyHook climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    currentState = commandState.Starting;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = commandState.Starting; // reset to starting state
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean moveToNextState = false;
    SmartDashboard.putString("ReachForBar", "Executing");

    switch (currentState){
      case Starting:
        // Just do it, don't check talon hooks
        moveToNextState = true;
        break;

      case PullHook:
        if (m_climber.moveHookToPosition(ClimberConstants.hookPullupPos, true)){
          moveToNextState = true;
        }
       break;
      
      case MoveArmForward:
          // move arm to vertical
          if (m_climber.moveArmToPosition(ClimberConstants.armPullupPos)){
            moveToNextState = true;
          }
          break;

      case RaiseHook:
        if (m_climber.moveHookToPosition(ClimberConstants.hookAboveFirstBarPos, true)){
          moveToNextState = true;
        }
        break;

      case Finished:
          break;

      default:
    }

    // move to next state?
    if (moveToNextState){ 
      currentState = currentState.getNext();
    }

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
    return false;
  }
}
