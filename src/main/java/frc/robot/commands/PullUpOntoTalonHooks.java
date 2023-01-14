/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberConstants;
import frc.robot.subsystems.LEDLights;

public class PullUpOntoTalonHooks extends CommandBase {
    private final Climber m_climber;
    private final LEDLights m_LEDLights;
private int countDown;
    private enum commandState {
      Starting,
      StraightenArm,
      PullTalonsAboveBar,
      ArmBackToTransferToTalons,
      TransferOntoTalons,
      CheckIfHooksLatched,
      Finished;

      public commandState getNext() {
        return this.ordinal() < commandState.values().length - 1
            ? commandState.values()[this.ordinal() + 1]
            : null;
      }
    }
    private static commandState currentState;


  public PullUpOntoTalonHooks(Climber climber, LEDLights LEDLights) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_LEDLights = LEDLights;
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

    switch (currentState){
      case Starting:
          // if either talon hooks are set, then don't do anything
          if (m_climber.LeftTalonHooked() || m_climber.RightTalonHooked()) {
            //currentState=commandState.Finished;
            m_LEDLights.lightsPurple();
          }
          else {
            moveToNextState=true;
          }
          break;

      case StraightenArm:
          // move arm to vertical
          m_LEDLights.lightsYellow();
          if (m_climber.moveArmToPosition(ClimberConstants.armPullupPos)){
            moveToNextState = true;
          }
          break;

      case PullTalonsAboveBar:
        // pull hook so talon hooks go past the bar
        m_LEDLights.lightsPurple();
        if (m_climber.moveHookToPosition(ClimberConstants.hookPullupPos, false)){
          moveToNextState = true;
        }
        break;

      case ArmBackToTransferToTalons:
        // reach climber arm back to swing robot forward
        m_LEDLights.lightsYellow();
        if (m_climber.moveArmToPosition(ClimberConstants.armTransferOntoTalonsPos)){
          countDown--;
          moveToNextState = (countDown <= 0);
          //moveToNextState = true;
        }
        else { countDown = 40; }
        break;

      case TransferOntoTalons:
        // extend hook up so weight is transferred to talon hooks
        m_LEDLights.lightsYellow();
        if (m_climber.moveHookToPosition(ClimberConstants.hookTransferToTalonsPos, true)){
          moveToNextState = true;
        }
        break;

      case CheckIfHooksLatched:
      // if both talon hooks are not set, then do another pull-up to try again
        if (m_climber.AllTalonsHooked()){
          m_LEDLights.lightsGreen();
          moveToNextState = true;         
        } 
        else { // failed transfer, try again
          m_LEDLights.lightsPurple();
          moveToNextState = true;
        }
        break;

      case Finished:
        m_climber.stopArm();
        m_climber.stopHook();
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
    if (interrupted){
      m_climber.stopHook();
      m_climber.stopArm(); 
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
