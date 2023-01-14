/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.Motors;
//import frc.robot.Constants.Solenoids;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Climber extends SubsystemBase {

private final WPI_TalonFX m_climber, m_climberArm;
private final DigitalInput m_talonHookLeft, m_talonHookRight;

public static final class ClimberConstants {
    // values for actual robot climber arm positions
    // starting position
    public static final double armsStartingPos = 0; // starting position
    public static final double armMaxReach = 118000; // 145000;
    public static final double hookStartingPos = 0;
    private static final double hookMaxReachPos = -340000;// -380000; //-406000; // 421000; 


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


// try buffering swing out of system
    // public static final double armBufferSwingPos = 20000;
    // public static final double hookBufferSwing = -100000;

    //public static final double hookReachPastNextBarPos = -402000;



    //tuned climber hook PID values
    public static final double kP = 0.04; 
    public static final double kI = 0.0001;
    public static final double kD = 0.0; 
    public static final double kIz = 4000; 
    public static final double kFF = 0;  //.000015; 

    public static final double kMaxOutput_Slow = 0.8; //0.75; 
    public static final double kMaxOutput_Fast = 0.9; //0.88;// 0.8; 
  }

public static final class ClimberArmConstants { 
  
  // tune the climber arm PID values
  public static final double kP = 0.04; 
  public static final double kI = 0.0001;
  public static final double kD = 0.0; 
  public static final double kIz = 4000; 
  public static final double kFF = 0;  //.000015; 
  public static final double kMaxOutput = 0.9; 
  // public static final double kMinOutput = -1;
  // public static final double maxRPM = 5700;  
  }

/**
   * Creates a new Climber.
   */
  public Climber() {
    super();

    m_talonHookLeft = new DigitalInput(DIOPorts.TALONHOOK_LEFT);
    m_talonHookRight = new DigitalInput(DIOPorts.TALONHOOK_RIGHT);

    m_climber = new WPI_TalonFX(Motors.CLIMBER_ONE);

    m_climber.configFactoryDefault();
    m_climber.setNeutralMode(NeutralMode.Brake);
    m_climber.setSelectedSensorPosition(ClimberConstants.hookStartingPos);
    m_climber.setInverted(TalonFXInvertType.Clockwise);

    m_climber.config_kP(0, ClimberConstants.kP);
    m_climber.config_kI(0, ClimberConstants.kI);
    m_climber.config_kD(0, ClimberConstants.kD);
    m_climber.config_IntegralZone(0, ClimberConstants.kIz);
    m_climber.config_kF(0, ClimberConstants.kFF);
    m_climber.configClosedLoopPeakOutput(0, ClimberConstants.kMaxOutput_Slow);

    // m_climber.config_kP(1, ClimberConstants.kP);
    // m_climber.config_kI(1, ClimberConstants.kI);
    // m_climber.config_kD(1, ClimberConstants.kD);
    // m_climber.config_IntegralZone(1, ClimberConstants.kIz);
    // m_climber.config_kF(1, ClimberConstants.kFF);
    // m_climber.configClosedLoopPeakOutput(1, ClimberConstants.kMaxOutput_Fast);


    m_climber.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    m_climber.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

    //m_climber.setlimits

    // m_climber.setIdleMode(IdleMode.kBrake);
    // m_climber.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    // m_climber.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

  //  m_climber.getEncoder().setPosition(kClimberMinPosition);

   // m_climber.setSoftLimit(SoftLimitDirection.kReverse, (float)kClimberMinPosition);
    //m_climber.setSoftLimit(SoftLimitDirection.kForward, (float)kClimberMaxPosition);
   // m_climber.enableSoftLimit(SoftLimitDirection.kReverse, true);
   // m_climber.enableSoftLimit(SoftLimitDirection.kForward, true);

    //armIsUp = true;

    m_climberArm = new WPI_TalonFX(Motors.CLIMBER_ARM);
    m_climberArm.configFactoryDefault();
    m_climberArm.setInverted(TalonFXInvertType.Clockwise);
    m_climberArm.setNeutralMode(NeutralMode.Brake);
    m_climberArm.setSelectedSensorPosition(ClimberConstants.armsStartingPos);

    // PID values for INTAKE_ARM
    m_climberArm.config_kP(0, ClimberArmConstants.kP);
    m_climberArm.config_kI(0, ClimberArmConstants.kI);
    m_climberArm.config_kD(0, ClimberArmConstants.kD);
    m_climberArm.config_IntegralZone(0, ClimberArmConstants.kIz);
    m_climberArm.config_kF(0, ClimberArmConstants.kFF);
    m_climberArm.configClosedLoopPeakOutput(0,ClimberArmConstants.kMaxOutput);

    m_climberArm.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    m_climberArm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

   // m_climberArm.configClearPositionOnLimitF(true,10);
    // m_climberArm.configClosedloopRamp(0.2)


    // m_IntakeArm.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
    // m_IntakeArm.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs)
    //m_IntakeArm.get

  }

  @Override
  public void periodic() {
    //m_climber.get
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("leftTalonHooked", LeftTalonHooked());
    SmartDashboard.putBoolean("rightTalonHooked", RightTalonHooked());
    
    SmartDashboard.putBoolean("climberArmRevLimit", armHitBackLimit());

    SmartDashboard.putNumber("climberArmPosition", m_climberArm.getSelectedSensorPosition());
    SmartDashboard.putBoolean("climberArmFwdLimit", armHitForwardLimit());
    SmartDashboard.putBoolean("climberArmRevLimit", armHitBackLimit());

    SmartDashboard.putNumber("climbHookPosition", m_climber.getSelectedSensorPosition());
    SmartDashboard.putBoolean("climbHookForwardLimit", hookHitForwardLimit());
    SmartDashboard.putBoolean("climbHookRevLimit", hookHitBackLimit());

   // if upper or lower limit switch is hit, then reset encoder position to upper or lower

   // Stopped resetting it here, only doing in in RunClimberManually

   //if (armHitForwardLimit()){
    //   m_climberArm.setSelectedSensorPosition(ClimberConstants.armMaxReach);
    // }
    // if (armHitBackLimit()){
    //   m_climberArm.setSelectedSensorPosition(ClimberConstants.armsStartingPos);
    // }

    // if (hookHitForwardLimit()){
    //   m_climber.setSelectedSensorPosition(ClimberConstants.armsStartingPos);
    // }
  }

  public void setArmSensorPosition(double armPosition){
    m_climberArm.setSelectedSensorPosition(armPosition);
  }

  public void setHookSensorPosition(double hookPosition){
    m_climber.setSelectedSensorPosition(ClimberConstants.armsStartingPos);
  }
  
  public boolean AllTalonsHooked(){
    return (LeftTalonHooked() && RightTalonHooked());
  }
  public boolean LeftTalonHooked(){
    return (! m_talonHookLeft.get());
  }
  public boolean RightTalonHooked(){
    return ( ! m_talonHookRight.get() );
  }

  

  public boolean moveHookToPositionSuperFast(double setPoint){
    m_climber.configClosedLoopPeakOutput(0, 1.0);

    m_climber.set(ControlMode.Position, setPoint);
    if (Math.abs(HookPosition() - setPoint) < 1000){
      return true;
    }

    // if reaching past physical limit, use the limit switch to report
    if (setPoint < ClimberConstants.hookMaxReachPos && hookHitBackLimit()){ return true; }
    if (setPoint > ClimberConstants.hookStartingPos && hookHitForwardLimit() ){ return true; }

    return false;
  }

  public boolean moveHookToPosition(double setPoint, boolean fastSpeed){
    if (fastSpeed){ 
      m_climber.configClosedLoopPeakOutput(0, ClimberConstants.kMaxOutput_Fast);
    }
    else { 
      m_climber.configClosedLoopPeakOutput(0, ClimberConstants.kMaxOutput_Slow); 
    }

    m_climber.set(ControlMode.Position, setPoint);
    if (Math.abs(HookPosition() - setPoint) < 1000){
      return true;
    }

    // if reaching past physical limit, use the limit switch to report
    if (setPoint < ClimberConstants.hookMaxReachPos && hookHitBackLimit()){ return true; }
    if (setPoint > ClimberConstants.hookStartingPos && hookHitForwardLimit() ){ return true; }
    if (hookHitForwardLimit() && setPoint > -5000) return true;
    return false;
  }


  public void extendHook(){
    //m_climber.set(ControlMode.Position, ClimberConstants.armExtendPos);
    m_climber.set(ControlMode.PercentOutput, -ClimberConstants.kMaxOutput_Slow);
  }

  public void pullHook(){
    //m_climber.set(ControlMode.Position, ClimberConstants.armHomePos);
    m_climber.set(ControlMode.PercentOutput, ClimberConstants.kMaxOutput_Slow);
  }
  
  public void stopHook(){
    m_climber.set(ControlMode.PercentOutput, 0);
  }
  public double HookPosition(){
    return m_climber.getSelectedSensorPosition();
  }
  public boolean hookHitForwardLimit(){
    return ( m_climber.getSensorCollection().isFwdLimitSwitchClosed()==1);
  }

  public boolean hookHitBackLimit(){
    return ( m_climber.getSensorCollection().isRevLimitSwitchClosed()==1);
  }



  public boolean moveArmToPosition(double setPoint){
    m_climberArm.set(ControlMode.Position, setPoint);   
    if (Math.abs(ArmPosition() - setPoint) < 2000){
      return true;
    }
    // if reaching past physical limit, use the limit switch to report
    if (setPoint > ClimberConstants.armMaxReach && armHitForwardLimit()){ return true; }
    if (setPoint < ClimberConstants.armsStartingPos && armHitBackLimit() ){ return true; }

    return false;
  }


   public void reachArmBack(){
    m_climberArm.set(ControlMode.PercentOutput, 0.3);
  }

  public void pullArmForward(){
    m_climberArm.set(ControlMode.PercentOutput, -0.3);  
  }

  public boolean armHitForwardLimit(){
    return ( m_climberArm.getSensorCollection().isFwdLimitSwitchClosed()==1);
  }

  public boolean armHitBackLimit(){
    return ( m_climberArm.getSensorCollection().isRevLimitSwitchClosed()==1);
  }

  public void stopArm(){
    m_climberArm.set(ControlMode.PercentOutput, 0);
  }

  public double ArmPosition(){
    return m_climberArm.getSelectedSensorPosition();
  }


  // public void allowAdditionalMovement(){
  //   m_climber.setSelectedSensorPosition((float)((kClimberMaxPosition - kClimberMinPosition)/2));
  // }
  // public void setToRetractedPosition(){
  //   m_climber.setSelectedSensorPosition((float)kClimberMinPosition);
  // }


}
