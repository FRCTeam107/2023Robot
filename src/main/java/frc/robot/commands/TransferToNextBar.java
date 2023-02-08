/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkyHook;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.SkyHook.ClimberArmConstants;
import frc.robot.subsystems.SkyHook.ClimberConstants;

public class TransferToNextBar extends CommandBase {
    private final SkyHook m_climber;
    private final LEDLights m_LEDLights;

    private final BooleanSupplier m_forceToRun;

    private enum commandState {
      Starting,
      ReleaseCurrentBar,
      // RaiseHookPunchNextBar,
      // BendArmToPunchNextBar,
      WaitForSwingToStop,
      BendArmBackToNextBar,
      // LowerHookBelowBar,
      ReachHookPastNextBar,
      RetractArmToTouchBar,
      PullHookToReleaseTalons,
      RetractArmForNextClimb,
      BufferSwingOut,
      Finished;

      public commandState getNext() {
        return this.ordinal() < commandState.values().length - 1
            ? commandState.values()[this.ordinal() + 1]
            : null;
      }
    }
    private static commandState currentState;
    private static int countDown;

  public TransferToNextBar(SkyHook climber, LEDLights LEDLights, BooleanSupplier _forceToRun) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_LEDLights = LEDLights;
    m_forceToRun = _forceToRun;

    currentState = commandState.Starting;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = commandState.Starting;
    countDown = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean moveToNextState = false;
    switch (currentState){
      case Starting:
        // if both talon hooks are NOT set, don't do anything!
        if (m_climber.AllTalonsHooked()){
          moveToNextState = true;
        }
        else if (m_forceToRun.getAsBoolean()){
          moveToNextState = true;
        }
        else {
          m_LEDLights.lightsPurple();
        }
        
        break;
      
      case ReleaseCurrentBar:
        // extend the hook past the current bar and to punch next one
        m_LEDLights.lightsYellow();
        if (m_climber.HookPosition() < ClimberConstants.hookReleasecurrentBar){
          moveToNextState =  true;
        }
        else {
          moveToNextState =  m_climber.moveHookToPosition(ClimberConstants.hookReleasecurrentBar, true);
        }
        
        if (moveToNextState) {
          countDown = 2; // 20ms loop * countdown timer;
        }
        break;
      
      // case RaiseHookPunchNextBar:
      //   // extend the hook past the current bar and to punch next one
      //   if (m_climber.moveHookToPosition(ClimberConstants.hookToPunchNextBar, true)){
      //     moveToNextState = true;
      //   }
      //   break;

      // case BendArmToPunchNextBar:
      //   if (m_climber.moveArmToPosition(ClimberConstants.armToPunchNextBar)){
      //     countDown = 100; // 20ms loop * countdown timer
      //     moveToNextState = true;
      //   }
      //   break;

      case WaitForSwingToStop:
        m_LEDLights.lightsYellow();
        countDown --;
        moveToNextState = (countDown<=0);
        break;

            // case LowerHookBelowBar:
      //   // extend the hook past the current bar and to punch next one
      //   m_LEDLights.lightsYellow();
      //   if (m_climber.moveHookToPosition(ClimberConstants.hookBelowNextBar, true)){
      //     moveToNextState = true;
      //   }
      //   break;

      case BendArmBackToNextBar:
        // move arm to reach backwards slightly past the next bar
        m_LEDLights.lightsYellow();

        boolean armReady = m_climber.moveArmToPosition(ClimberConstants.armReachPastNextBar);
        boolean hookReady = m_climber.moveHookToPosition(ClimberConstants.hookBelowNextBar, true);
        moveToNextState = (armReady && hookReady);
        break;
        
      case ReachHookPastNextBar:
        // extend the hook past the next bar
        m_LEDLights.lightsYellow();

        if (m_climber.moveHookToPosition(ClimberConstants.hookPastNextBar, true)){
          moveToNextState = true;
        }
        break;

      case RetractArmToTouchBar:
        // now bring arm forward to make arm touch the bar
        m_LEDLights.lightsYellow();

        if (m_climber.moveArmToPosition(ClimberConstants.armHugNextBar)) {
          moveToNextState = true;
          countDown = 20; // 20ms loop * countdown timer;
        }
        // todo:  consider a pause here to settle down?
        break;

      case PullHookToReleaseTalons:
        // pull the hook far enough to release the talons hooks
        m_LEDLights.lightsYellow();

        countDown --;
        if (countDown <= 0){
          if (m_climber.moveHookToPosition(ClimberConstants.hookPullTalonsOffBar, false) ) {
            moveToNextState = true;
          } 
        }
        break;

      case RetractArmForNextClimb:
        if (m_climber.moveArmToPosition(ClimberConstants.armTransferOntoTalonsPos)) {
          moveToNextState = true;
        }
        break;

      case BufferSwingOut:
        moveToNextState = true;
        break;
        // armReady = m_climber.moveArmToPosition(ClimberConstants.armBufferSwingPos);
        // hookReady = m_climber.moveHookToPosition(ClimberConstants.hookBufferSwing);
        // moveToNextState = armReady && hookReady;
        // break;
     
      case Finished:
        if (m_climber.LeftTalonHooked() || m_climber.RightTalonHooked()){
          m_LEDLights.lightsPurple();
        }
        else {
          m_LEDLights.lightsGreen();
        }
        break;

      default:
        //break;
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
