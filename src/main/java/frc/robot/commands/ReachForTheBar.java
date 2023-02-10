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
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.SkyHook.ExtensionConstants;

public class ReachForTheBar extends CommandBase {
    private final SkyHook m_climber;
    private final LEDLights m_LEDLights;

    private enum commandState {
      Starting,
      StraightenArmAndLiftHook,
      Finished;

      public commandState getNext() {
        return this.ordinal() < commandState.values().length - 1
            ? commandState.values()[this.ordinal() + 1]
            : null;
      }
    }
    private static commandState currentState;



  public ReachForTheBar(SkyHook climber, LEDLights LEDLights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_LEDLights = LEDLights;
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
    //SmartDashboard.putString("ReachForBar", "Executing");

    switch (currentState){
      case Starting:
        // if any talon hooks are set, don't do anything!
        if (m_climber.LeftTalonHooked() || m_climber.RightTalonHooked()){
          currentState = commandState.Finished;
        }
        else {
          moveToNextState = true;
        }
        break;

      case StraightenArmAndLiftHook:
        m_LEDLights.lightsYellow();
        // move arm to vertical and lift hook so it is above the first bar (at same time)
        boolean armReady = m_climber.moveArmToPosition(ExtensionConstants.armFirstBarPos);
        boolean hookReady = m_climber.moveHookToPositionSuperFast(ExtensionConstants.hookAboveFirstBarPos);   
        moveToNextState = (hookReady && armReady);
       break;
        
      case Finished:
        m_LEDLights.lightsGreen();
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
