/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;

public class SkyHook extends SubsystemBase {

  private final CANSparkMax m_ExtensionMotor, m_FlipFlopMotor;
  private final SparkMaxPIDController m_ExtensionPID, m_FlipFlopPID;
  private final DoubleSolenoid m_clapper;

//private final DigitalInput m_talonHookLeft, m_talonHookRight;

public static final class ExtensionConstants {
    // PID values
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;

    // values for actual robot climber arm positions
    // starting position
    public static final double armsStartingPos = 0; // starting position
    public static final double armMaxReach = 118000; // 145000;
    public static final double hookStartingPos = 0;
    //private static final double hookMaxReachPos = -340000;// -380000; //-406000; // 421000; 


    // reach for the bar
    public static final double armFirstBarPos = 4000;//6000; // reach up to bar
    public static final double hookAboveFirstBarPos = -320000; //-339000; // -234000;

    
    // do a pull-up
    public static final double armPullupPos = 4000; //4000;// 1000; // keep arm stiff during pullup
    // note: pullup position is beyond limit of the hook, need to pull right to limit switch
    public static final double hookPullupPos = -4000;//-2000; // -5000; // + 20000); // position to pullup and get talons to "hook"

    // release onto talons
    public static final double armTransferOntoTalonsPos = 28000;
    public static final double hookTransferToTalonsPos = -29000; //-33200; //-29800;

    // steps to get to next bar:
    public static final double hookReleasecurrentBar = -65000; //-52000; //-29800;

    // public static final double armToPunchNextBar = 97800; //118000;
    // public static final double hookToPunchNextBar = -330000;

    public static final double hookBelowNextBar = -220000;
    public static final double armReachPastNextBar = 160000; //140000;

    // note: position is beyond limit of the hook, need to pull right to limit switch
    public static final double hookPastNextBar = -350000; //-378000;// (hookMaxReachPos - 8000);

    public static final double armHugNextBar = 125000; // 120000;
    public static final double hookPullTalonsOffBar = -233000; //-270000;
  }

public static final class FlipFlopConstants { 
  
    // PID values
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;

  }

/**
   * Creates a new SkyHook.
   */
  public SkyHook() {
    super();

    m_ExtensionMotor = new CANSparkMax(Motors.SkyhookExtension, MotorType.kBrushless);
    m_ExtensionMotor.restoreFactoryDefaults();
    // Reduce CAN bus traffic
    m_ExtensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_ExtensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_ExtensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_clapper = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 4, 5);

    m_ExtensionPID = m_ExtensionMotor.getPIDController();
    m_ExtensionPID.setP(ExtensionConstants.kP);
    m_ExtensionPID.setI(ExtensionConstants.kI);
    m_ExtensionPID.setD(ExtensionConstants.kD);
    m_ExtensionPID.setIZone(ExtensionConstants.kIz);
    m_ExtensionPID.setFF(ExtensionConstants.kFF);
    m_ExtensionPID.setOutputRange(ExtensionConstants.kMinOutput, ExtensionConstants.kMaxOutput);

    m_FlipFlopMotor = new CANSparkMax(Motors.SkyhookFlipFlop, MotorType.kBrushless);
    m_FlipFlopMotor.restoreFactoryDefaults();
    // reduce communication on CAN bus
    m_FlipFlopMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_FlipFlopMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_FlipFlopMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_FlipFlopPID = m_FlipFlopMotor.getPIDController();
    m_FlipFlopPID.setP(FlipFlopConstants.kP);
    m_FlipFlopPID.setI(FlipFlopConstants.kI);
    m_FlipFlopPID.setD(FlipFlopConstants.kD);
    m_FlipFlopPID.setIZone(FlipFlopConstants.kIz);
    m_FlipFlopPID.setFF(FlipFlopConstants.kFF);
    m_FlipFlopPID.setOutputRange(FlipFlopConstants.kMinOutput, FlipFlopConstants.kMaxOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // output values that show on driver screen dashboard, or are used in LED lights
    // SmartDashboard.putBoolean("leftTalonHooked", LeftTalonHooked());
    // SmartDashboard.putBoolean("rightTalonHooked", RightTalonHooked());
    
    // SmartDashboard.putBoolean("climberArmRevLimit", armHitBackLimit());

    // SmartDashboard.putNumber("climberArmPosition", m_climberArm.getSelectedSensorPosition());
    // SmartDashboard.putBoolean("climberArmFwdLimit", armHitForwardLimit());
    // SmartDashboard.putBoolean("climberArmRevLimit", armHitBackLimit());

    // SmartDashboard.putNumber("climbHookPosition", m_climber.getSelectedSensorPosition());
    // SmartDashboard.putBoolean("climbHookForwardLimit", hookHitForwardLimit());
    // SmartDashboard.putBoolean("climbHookRevLimit", hookHitBackLimit());
  }

  public void squeeze(){
    SmartDashboard.putString("skyHookArm", "Squeeze");
    m_clapper.set(kForward);
  }

  public void releeve(){
    SmartDashboard.putString("skyHookArm", "Release");
     m_clapper.set(kReverse);
  }
  // public void setArmSensorPosition(double armPosition){
  //   m_climberArm.setSelectedSensorPosition(armPosition);
  // }

  // public void setHookSensorPosition(double hookPosition){
  //   m_climber.setSelectedSensorPosition(ExtensionConstants.armsStartingPos);
  // }
  
  // public boolean AllTalonsHooked(){
  //   return (LeftTalonHooked() && RightTalonHooked());
  // }
  // public boolean LeftTalonHooked(){
  //   return (! m_talonHookLeft.get());
  // }
  // public boolean RightTalonHooked(){
  //   return ( ! m_talonHookRight.get() );
  // }

  

  // public boolean moveHookToPositionSuperFast(double setPoint){
  //   m_climber.configClosedLoopPeakOutput(0, 1.0);

  //   m_climber.set(ControlMode.Position, setPoint);
  //   if (Math.abs(HookPosition() - setPoint) < 1000){
  //     return true;
  //   }

  //   // if reaching past physical limit, use the limit switch to report
  //   if (setPoint < ExtensionConstants.hookMaxReachPos && hookHitBackLimit()){ return true; }
  //   if (setPoint > ExtensionConstants.hookStartingPos && hookHitForwardLimit() ){ return true; }

  //   return false;
  // }


  // public void extendHook(){
  //   //m_climber.set(ControlMode.Position, ClimberConstants.armExtendPos);
  //   m_climber.set(ControlMode.PercentOutput, -ExtensionConstants.kMaxOutput_Slow);
  // }

  // public void pullHook(){
  //   //m_climber.set(ControlMode.Position, ClimberConstants.armHomePos);
  //   m_climber.set(ControlMode.PercentOutput, ExtensionConstants.kMaxOutput_Slow);
  // }
  
  // public void stopHook(){
  //   m_climber.set(ControlMode.PercentOutput, 0);
  // }
  // public double HookPosition(){
  //   return m_climber.getSelectedSensorPosition();
  // }
  // public boolean hookHitForwardLimit(){
  //   return ( m_climber.getSensorCollection().isFwdLimitSwitchClosed()==1);
  // }

  // public boolean hookHitBackLimit(){
  //   return ( m_climber.getSensorCollection().isRevLimitSwitchClosed()==1);
  // }



  // public boolean moveArmToPosition(double setPoint){
  //   m_climberArm.set(ControlMode.Position, setPoint);   
  //   if (Math.abs(ArmPosition() - setPoint) < 2000){
  //     return true;
  //   }
  //   // if reaching past physical limit, use the limit switch to report
  //   if (setPoint > ExtensionConstants.armMaxReach && armHitForwardLimit()){ return true; }
  //   if (setPoint < ExtensionConstants.armsStartingPos && armHitBackLimit() ){ return true; }

  //   return false;
  // }


  //  public void reachArmBack(){
  //   m_climberArm.set(ControlMode.PercentOutput, 0.3);
  // }

  // public void pullArmForward(){
  //   m_climberArm.set(ControlMode.PercentOutput, -0.3);  
  // }

  // public boolean armHitForwardLimit(){
  //   return ( m_climberArm.getSensorCollection().isFwdLimitSwitchClosed()==1);
  // }

  // public boolean armHitBackLimit(){
  //   return ( m_climberArm.getSensorCollection().isRevLimitSwitchClosed()==1);
  // }

  // public void stopArm(){
  //   m_climberArm.set(ControlMode.PercentOutput, 0);
  // }

  // public double ArmPosition(){
  //   return m_climberArm.getSelectedSensorPosition();
  // }

}
