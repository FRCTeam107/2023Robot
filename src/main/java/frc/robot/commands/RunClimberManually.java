package frc.robot.commands;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkyHook;
import frc.robot.subsystems.SkyHook.ExtensionConstants;

public class RunClimberManually extends CommandBase {

  private final SkyHook m_Climber;
  //private final XboxController leftCcontroller;
  private final Joystick m_Joystick;
  // private final Joystick rightController;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.

  public RunClimberManually(SkyHook _climber, Joystick controller) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    m_Climber = _climber;
    m_Joystick = controller;
    addRequirements(m_Climber);
  }

  @Override
  public void execute() {
    int hookMove = m_Joystick.getPOV();
    double armMove = m_Joystick.getY();


    if (m_Climber.armHitBackLimit()){
      m_Climber.setArmSensorPosition(ExtensionConstants.armsStartingPos);
    }

    if (m_Climber.hookHitForwardLimit()){
      m_Climber.setHookSensorPosition(ExtensionConstants.armsStartingPos);
    }
    
    if (hookMove==0 || hookMove==45 || hookMove==315){
      m_Climber.extendHook();
    }
    else if (hookMove==180 || hookMove==135 || hookMove==225){
      m_Climber.pullHook();
    }
    else {
      m_Climber.stopHook();
    }

    if (armMove < -0.3){
      m_Climber.pullArmForward();
    }
    else if (armMove > 0.3){
      m_Climber.reachArmBack();
    }
    else {
      m_Climber.stopArm();
    }
  }

}
