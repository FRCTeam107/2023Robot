/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SkyHook extends SubsystemBase {

  private final CANSparkMax m_ExtensionMotor, m_ArmMotor, m_WristMotor, m_IntakeMotor;
  private final SparkMaxPIDController m_ExtensionPID, m_ArmPID, m_WristPID, m_IntakePID;

  private CANSparkMax.ControlType m_ExtensionCtrlType, m_ArmCtrlType, m_WristCtrlType, m_IntakeCtrlType;
  private double m_ExtensionSetpoint, m_ArmSetpoint, m_WristSetpoint, m_IntakeSetpoint;
  public static final class ArmFlip{
    public static final double BACK = 20;
    public static final double FORWARD = -10;
    public static final double HOME = 0;
    }

static final class ExtensionConstants {
    // PID values
    static final double kP = 0.4096;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 8000;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 0.05;
    static final double kMinOutput = -0.05;

    // values for actual robot climber arm positions
    // starting position
    // public static final double armsStartingPos = 0; // starting position
    // public static final double armMaxReach = 118000; // 145000;
    // public static final double hookStartingPos = 0;
    //private static final double hookMaxReachPos = -340000;// -380000; //-406000; // 421000; 


    // reach for the bar
    // public static final double armFirstBarPos = 4000;//6000; // reach up to bar
    // public static final double hookAboveFirstBarPos = -320000; //-339000; // -234000;

    
    // do a pull-up
    // public static final double armPullupPos = 4000; //4000;// 1000; // keep arm stiff during pullup
    // // note: pullup position is beyond limit of the hook, need to pull right to limit switch
    // public static final double hookPullupPos = -4000;//-2000; // -5000; // + 20000); // position to pullup and get talons to "hook"

    // // release onto talons
    // public static final double armTransferOntoTalonsPos = 28000;
    // public static final double hookTransferToTalonsPos = -29000; //-33200; //-29800;

    // // steps to get to next bar:
    // public static final double hookReleasecurrentBar = -65000; //-52000; //-29800;

    // // public static final double armToPunchNextBar = 97800; //118000;
    // // public static final double hookToPunchNextBar = -330000;

    // public static final double hookBelowNextBar = -220000;
    // public static final double armReachPastNextBar = 160000; //140000;

    // // note: position is beyond limit of the hook, need to pull right to limit switch
    // public static final double hookPastNextBar = -350000; //-378000;// (hookMaxReachPos - 8000);

    // public static final double armHugNextBar = 125000; // 120000;
    // public static final double hookPullTalonsOffBar = -233000; //-270000;
  }

  static final class ArmConstants { 
    // PID values
    static final double kP = 0.4096;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 8000;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 0.2;
    static final double kMinOutput = -0.2;
  }
  static final class WristConstants { 
    // PID values
    static final double kP = 0.4096;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 8000;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 0.2;
    static final double kMinOutput = -0.2;
  }
  static final class IntakeConstants { 
    // PID values
    static final double kP = 0.4096;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 8000;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 0.2;
    static final double kMinOutput = -0.2;
  }
/**
   * Creates a new SkyHook.
   */
  public SkyHook() {
    super();

    m_ExtensionCtrlType = ControlType.kDutyCycle;
    m_ArmCtrlType = ControlType.kDutyCycle;
    m_WristCtrlType = ControlType.kDutyCycle;
    m_IntakeCtrlType = ControlType.kDutyCycle;
    m_ExtensionSetpoint = 0;
    m_ArmSetpoint = 0;
    m_WristSetpoint = 0;
  
    m_ExtensionMotor = new CANSparkMax(Motors.SKYHOOK_EXTENDER, MotorType.kBrushless);
    m_ExtensionMotor.restoreFactoryDefaults();
    m_ExtensionMotor.setIdleMode(IdleMode.kBrake);
    m_ExtensionMotor.getEncoder().setPosition(ArmFlip.HOME);

    // Reduce CAN bus traffic
    m_ExtensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_ExtensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_ExtensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_ExtensionPID = m_ExtensionMotor.getPIDController();
    m_ExtensionPID.setP(ExtensionConstants.kP);
    m_ExtensionPID.setI(ExtensionConstants.kI);
    m_ExtensionPID.setD(ExtensionConstants.kD);
    m_ExtensionPID.setIZone(ExtensionConstants.kIz);
    m_ExtensionPID.setFF(ExtensionConstants.kFF);
    m_ExtensionPID.setOutputRange(ExtensionConstants.kMinOutput, ExtensionConstants.kMaxOutput);

    // Skyhook "Arm" to make it flip-flop
    m_ArmMotor = new CANSparkMax(Motors.SKYHOOK_ARM, MotorType.kBrushless);
    m_ArmMotor.restoreFactoryDefaults();
    m_ArmMotor.setIdleMode(IdleMode.kBrake);
    m_ArmMotor.getEncoder().setPosition(0);

    // reduce communication on CAN bus
    m_ArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_ArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_ArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_ArmPID = m_ArmMotor.getPIDController();
    m_ArmPID.setP(ArmConstants.kP);
    m_ArmPID.setI(ArmConstants.kI);
    m_ArmPID.setD(ArmConstants.kD);
    m_ArmPID.setIZone(ArmConstants.kIz);
    m_ArmPID.setFF(ArmConstants.kFF);
    m_ArmPID.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

    // Wrist to bend the intake head
    m_WristMotor = new CANSparkMax(Motors.SKYHOOK_WRIST, MotorType.kBrushless);
    m_WristMotor.restoreFactoryDefaults();
    m_WristMotor.setIdleMode(IdleMode.kBrake);
    m_WristMotor.getEncoder().setPosition(0);

    // reduce communication on CAN bus
    m_WristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_WristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_WristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_WristPID = m_WristMotor.getPIDController();
    m_WristPID.setP(WristConstants.kP);
    m_WristPID.setI(WristConstants.kI);
    m_WristPID.setD(WristConstants.kD);
    m_WristPID.setIZone(WristConstants.kIz);
    m_WristPID.setFF(WristConstants.kFF);
    m_WristPID.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);

    // intake motor
    m_IntakeMotor = new CANSparkMax(Motors.SKYHOOK_INTAKE, MotorType.kBrushless);
    m_IntakeMotor.restoreFactoryDefaults();
    m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    m_IntakeMotor.getEncoder().setPosition(0);
    // reduce communication on CAN bus
    m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_IntakePID = m_IntakeMotor.getPIDController();
    m_IntakePID.setP(IntakeConstants.kP);
    m_IntakePID.setI(IntakeConstants.kI);
    m_IntakePID.setD(IntakeConstants.kD);
    m_IntakePID.setIZone(IntakeConstants.kIz);
    m_IntakePID.setFF(IntakeConstants.kFF);
    m_IntakePID.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // output values that show on driver screen dashboard, or are used in LED lights
    SmartDashboard.putNumber("SkyHookArm", GetArmPosition());
    SmartDashboard.putNumber("SkyHookExtension", GetExtensionPosition());
    SmartDashboard.putNumber("SkyHookWrist", GetWristPosition());
    //SmartDashboard.putNumber("SkyHookExtension", GetExtensionPosition());

    // TODO: put code here to prevent moving arm through robot if extension or wrist in unsafe position
    // ideally, the code will move the wrist & arm to safely pass through robot

    m_ArmPID.setReference(m_ArmSetpoint, m_ArmCtrlType);
    m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
    m_WristPID.setReference(m_WristSetpoint, m_WristCtrlType);
    m_IntakePID.setReference(m_IntakeSetpoint, m_IntakeCtrlType);
  }

  // methods for Arm motor
  public void SetArmPosition(double position){
    m_ArmCtrlType = ControlType.kPosition;
    m_ArmSetpoint = position;
   }
   public void SetArmVelocity(double velocity){
    m_ArmCtrlType = ControlType.kVelocity;
    m_ArmSetpoint = velocity;
   }
   public void SetArmSmartMotion(double position){
    m_ArmCtrlType = ControlType.kSmartMotion;
    m_ArmSetpoint = position;
   }
   public void SetArmPower(double percent){
    m_ArmCtrlType = ControlType.kDutyCycle;
    m_ArmSetpoint = percent;
   }
   public double GetArmPosition(){
    return m_ArmMotor.getEncoder().getPosition();
   }
   public double GetArmVelocity(){
    return m_ArmMotor.getEncoder().getVelocity();
   }
   
  // methods for Extension motor
   public void SetExtensionPosition(double position){
    m_ExtensionCtrlType = ControlType.kPosition;
    m_ExtensionSetpoint = position;
   }
   public void SetExtensionVelocity(double velocity){
    m_ArmCtrlType = ControlType.kVelocity;
    m_ArmSetpoint = velocity;
   }
   public void SetExtensionSmartMotion(double position){
    m_ArmCtrlType = ControlType.kSmartMotion;
    m_ArmSetpoint = position;
   }
   public void SetExtensionPower(double percent){
    m_ArmCtrlType = ControlType.kDutyCycle;
    m_ArmSetpoint = percent;
   }
   public double GetExtensionPosition(){
    return m_ArmMotor.getEncoder().getPosition();
   }
   public double GetExtensionVelocity(){
    return m_ArmMotor.getEncoder().getVelocity();
   }

  // methods for Wrist motor   
   public void SetWristPosition(double position){
    m_WristCtrlType = ControlType.kPosition;
    m_WristSetpoint = position;
   }
   public void SetWristVelocity(double velocity){
    m_WristCtrlType = ControlType.kVelocity;
    m_WristSetpoint = velocity;
   }
   public void SetWristSmartMotion(double position){
    m_WristCtrlType = ControlType.kSmartMotion;
    m_WristSetpoint = position;
   }
   public void SetWristPower(double percent){
    m_WristCtrlType = ControlType.kDutyCycle;
    m_WristSetpoint = percent;
   }
   public double GetWristPosition(){
    return m_WristMotor.getEncoder().getPosition();
   }
   public double GetWristVelocity(){
    return m_WristMotor.getEncoder().getVelocity();
   }

   // methods for Intake motor
   public void SetIntakePosition(double position){
    m_IntakeCtrlType = ControlType.kPosition;
    m_IntakeSetpoint = position; 
   }
   public void SetIntakeVelocity(double velocity){
    m_IntakeCtrlType = ControlType.kVelocity;
    m_IntakeSetpoint = velocity;
   }
   public void SetSmartMotion(double position){
    m_IntakeCtrlType = ControlType.kSmartMotion;
    m_IntakeSetpoint = position;
   }
   public void SetIntakePower(double percent){
    m_IntakeCtrlType = ControlType.kDutyCycle;
    m_IntakeSetpoint = percent;
   }
   public double GetIntakePosition(){
    return m_IntakeMotor.getEncoder().getPosition();
   }
   public double GetIntakeVelocity(){
    return m_IntakeMotor.getEncoder().getVelocity();
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
