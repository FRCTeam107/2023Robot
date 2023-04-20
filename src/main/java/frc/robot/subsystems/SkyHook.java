/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;
// import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.DataRecorder.datapoint;

public class SkyHook extends SubsystemBase {

  private final CANSparkMax m_ExtensionMotor;
  private final WPI_TalonFX m_WristMotor, m_IntakeMotor, m_ArmMotor;//, m_ArmMotor2;
  private final CANifier m_Canifier;

  private ControlMode m_WristCtrlType, m_IntakeCtrlType, m_ArmCtrlType;
  private boolean inManualControlMode = false;

  private final SparkMaxPIDController m_ExtensionPID;
  private CANSparkMax.ControlType m_ExtensionCtrlType;

  private int powerEjectTimer = 0;

  private double m_ExtensionSetpoint, m_ArmSetpoint, m_WristSetpoint, m_IntakeSetpoint;
  //private double m_ArmLastPosition;//, m_ExtensionHoldSetpoint, m_WristHoldSetpoint;

  // public static final class ArmPositions{
  //   static final double UPPERLIMIT = 20; // maximum value for position
  //   public static final double BACK = 19;
  //   public static final double FORWARD = -12;
  //   public static final double STARTPOSITION = -4.8;
  //   static final double LOWERLIMIT = -20; // minimum value for position
  //   }
  public static final class ArmPositions{
      static final double MINLIMIT = -3000; //175000; // minimum value for position
      static final double MAXLIMIT = 3000; //-220000; // maximum value for position
      public static final double FULLFORWARD = -2500; //117000;
     //static final double UNSAFEPOSITIONMAX = 1; //1000; // upper point where not safe to extend elevator, and wrist must fold up
      public static final double STARTPOSITION = 0;
      //static final double UNSAFEPOSITIONMIN= -1;// -30000; // lower point where not safe to extend elevator, and wrist must fold up
      public static final double FULLBACK = 2500; //-165000;
      

      public static final double GROUNDPICKUP_FRONT = -117;//2000;
      public static final double UPRIGHTCONE_FRONT = -300;//2000;
      public static final double DRIVING = -10; //2000
      public static final double FEEDERPICKUP_FRONT = -900;//60000;
      //public static final double GROUNDSCORE_FRONT = -10;  //2000;
      public static final double TIER2SCORE_FRONT = -2050;//2200; // 111000;
      public static final double TIER3SCORE_FRONT = -2350;//2535; // 130000;

      public static final double TIER2SCORE_BACK = 2100; //-162000;
      public static final double TIER3SCORE_BACK = 2450; //-172000;
      }
  public static final class ExtensionPositions{
    static final double RETRACTLIMIT = 21;//28;//51;  //0; // actual limit (upper limit switch hit)
    public static final double RETRACTED = 20.75;//27.5;//49;//-5; // advertised retracted position
    static final double STARTPOSITION = 0;//-85;
    public static final double EXTENDED = -38.5; //-61;//-100;//-100;
    static final double EXTENDLIMIT = -39;//-50;//-61.5;//-115; // fully extended position
    public static final double GROUNDPICKUP_FRONT = 5;//7.5;//13;
    public static final double UPRIGHTCONE_FRONT = 19;//18;//27.5;
    public static final double DRIVING = RETRACTED;
    public static final double FEEDERPICKUP_FRONT = RETRACTED;
    // public static final double GROUNDSCORE_FRONT = 5;
    public static final double TIER2SCORE_FRONT = 5.5;// -2.9; //-4.5;//-10;
    public static final double TIER3SCORE_FRONT = -38.5;//-62;//-115;

    public static final double TIER2SCORE_BACK = 6.5;//11.5; //20;
    public static final double TIER3SCORE_BACK = -35.5; //-61; //-110;

    //encoder position doesn't match setpoint for some reason
    static final double SAFESETPOINTMIN = 13;//22;//47;
    static final double SAFEPOSITIONMIN = 13;//28.5;//58; //setpoint=48
  }
  public static final class WristPositions{
    static final double MINLIMIT = -1350;//-1500; //9000; // actual limit (upper limit switch hit)
    public static final double FRONTFOLDUP = -1280;//-1500;//-8600; // advertised retracted position
    static final double STARTPOSITION = 0;//-9000;//-13000;
    public static final double BACKFOLDUP = 16500;
    static final double MAXLIMIT = 16500;//-9000; // fully extended position
    public static final double GROUNDPICKUP_FRONT = 2200;
    public static final double UPRIGHTCONE_FRONT = 4000;
    public static final double DRIVING = 10;
    public static final double FEEDERPICKUP_FRONT = -700;
    public static final double GROUNDSCORE_FRONT = -1000;
    public static final double TIER2SCORE_FRONT = 10500;
    public static final double TIER3SCORE_FRONT = 11000;

    public static final double TIER2SCORE_BACK = 8000;
    public static final double TIER3SCORE_BACK = 6000;
  }
static final class ExtensionConstants {
    // PID values
    static final double kP = 0.035; //0.05;
    static final double kI = 0.00001;
    static final double kD = 0;
    static final double kIz = 100;
    static final double kFF = 0.001;//0.01;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
  }

  static final class ArmConstants { 
    // PID values
    static final double kP = 0.8;//0.7; //0.2;//0.03;
    static final double kI = 0.0003; //1e-6;
    static final double kD = 0;
    static final double kIz = 800;//0;
    static final double kFF = 0.107;// 0.08;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
    static final double kCruiseVelocity = 40000;
    static final double kMaxAccel = 3000;
  }
  static final class WristConstants { 
    // PID values
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = 0.08;//.000015;
    static final double kMaxOutput = 0.3;
    static final double kMinOutput = -0.3;
  }
  static final class IntakeConstants { 
    // PID values
    static final double kP = 0.4096;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
  }
/**
   * Creates a new SkyHook.
   */
  public SkyHook() {
    super();

    // these smart dashboard values allow manual adjustments to setpoints
    // when using the related SkyHook_MoveXXXXX commands 
    double var;
    var = SmartDashboard.getNumber("Arm To", 0.0);
    SmartDashboard.putNumber("Arm To", var);
    var = SmartDashboard.getNumber("Wrist To", 0.0);
    SmartDashboard.putNumber("Wrist To", var);
    var = SmartDashboard.getNumber("Extension To", 0.0);
    SmartDashboard.putNumber("Extension To", var);

    m_ExtensionCtrlType = ControlType.kDutyCycle;
    m_ArmCtrlType = ControlMode.PercentOutput;
    m_WristCtrlType = ControlMode.PercentOutput;// ControlType.kDutyCycle;
    m_IntakeCtrlType = ControlMode.PercentOutput;//ControlType.kDutyCycle;
    m_ExtensionSetpoint = 0;
    m_ArmSetpoint = 0;
    m_WristSetpoint = 0;
  
    m_ExtensionMotor = new CANSparkMax(Motors.SKYHOOK_EXTENDER, MotorType.kBrushless);
    m_ExtensionMotor.restoreFactoryDefaults();
    m_ExtensionMotor.setIdleMode(IdleMode.kBrake);
    m_ExtensionMotor.getEncoder().setPosition(ExtensionPositions.STARTPOSITION);
    m_ExtensionMotor.setSmartCurrentLimit(20);

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
    m_Canifier = new CANifier(Motors.SKYHOOK_CANIFIER);
    m_Canifier.configFactoryDefault();
    //m_Canifier.setQuadraturePosition(0, 30);

    m_ArmMotor = new WPI_TalonFX(Motors.SKYHOOK_ARM);
    m_ArmMotor.configFactoryDefault();
    m_ArmMotor.config_kP(0, ArmConstants.kP);
    m_ArmMotor.config_kI(0, ArmConstants.kI);
    m_ArmMotor.config_kD(0, ArmConstants.kD);
    m_ArmMotor.config_IntegralZone(0, ArmConstants.kIz);
    m_ArmMotor.config_kF(0, ArmConstants.kFF);
 
    TalonFXConfiguration armTalonFXConfiguration = new TalonFXConfiguration();
    armTalonFXConfiguration.slot0.kP = ArmConstants.kP;
    armTalonFXConfiguration.slot0.kI = ArmConstants.kI;
    armTalonFXConfiguration.slot0.kD = ArmConstants.kD;
    armTalonFXConfiguration.slot0.integralZone = ArmConstants.kIz;
    armTalonFXConfiguration.slot0.kF = ArmConstants.kFF;
    armTalonFXConfiguration.slot0.closedLoopPeakOutput = ArmConstants.kMaxOutput;
    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    armTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = m_Canifier.getDeviceID();
    armTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANifier_Quadrature; 
  
    armTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    m_ArmMotor.configAllSettings(armTalonFXConfiguration);
    m_ArmMotor.configClosedLoopPeakOutput(0, ArmConstants.kMaxOutput);
    m_ArmMotor.configPeakOutputForward(ArmConstants.kMaxOutput);
    m_ArmMotor.configPeakOutputReverse(ArmConstants.kMinOutput);
    m_ArmMotor.configMotionAcceleration(ArmConstants.kMaxAccel);
    m_ArmMotor.configMotionCruiseVelocity(ArmConstants.kCruiseVelocity);
    
    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    //m_ArmMotor.configRemoteFeedbackFilter(m_Canifier.getDeviceID(), RemoteSensorSource.CANifier_Quadrature, 0);
    //m_ArmMotor.configRe .primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    m_ArmMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    //m_ArmMotor.configSelectedFeedbackSensor(m_Canifier.getDeviceID(), RemoteSensorSource.CANifier_Quadrature, 0);
    m_ArmMotor.setSelectedSensorPosition(ArmPositions.STARTPOSITION);
   // m_ArmMotor.configuration

    // m_ArmMotor2 = new WPI_TalonFX(Motors.SKYHOOK_ARM2);
    // m_ArmMotor2.configFactoryDefault();
    // m_ArmMotor2.follow(m_ArmMotor);
    // m_ArmMotor2.setInverted(false);
    // // reduce communication on CAN bus
    // m_ArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // m_ArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    // m_ArmMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    // m_ArmPID = m_ArmMotor.getPIDController();
    // m_ArmPID.setP(ArmConstants.kP);
    // m_ArmPID.setI(ArmConstants.kI);
    // m_ArmPID.setD(ArmConstants.kD);
    // m_ArmPID.setIZone(ArmConstants.kIz);
    // m_ArmPID.setFF(ArmConstants.kFF);
    // m_ArmPID.setSmartMotionMaxAccel(ArmConstants.kMaxAccel, 0);
    // m_ArmPID.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, 0);
    // m_ArmPID.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
    // m_ArmMotor.burnFlash();

    // Wrist to bend the intake head
    m_WristMotor = new WPI_TalonFX(Motors.SKYHOOK_WRIST);
    m_WristMotor.configFactoryDefault();
    m_WristMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    m_WristMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    m_WristMotor.setSelectedSensorPosition(WristPositions.STARTPOSITION);
    m_WristMotor.setNeutralMode(NeutralMode.Brake);
    // m_shootbottom.configClosedloopRamp(0.3);
    // m_shoottop.configClosedloopRamp(0.3);
;
    // setup PID closed-loop values
    m_WristMotor.config_kP(0, WristConstants.kP);
    m_WristMotor.config_kI(0, WristConstants.kI);
    m_WristMotor.config_kD(0, WristConstants.kD);
    m_WristMotor.config_IntegralZone(0, WristConstants.kIz);
    m_WristMotor.config_kF(0, WristConstants.kFF);
    m_WristMotor.configClosedLoopPeakOutput(0, WristConstants.kMaxOutput);
    m_WristMotor.configPeakOutputForward(WristConstants.kMaxOutput);
    m_WristMotor.configPeakOutputReverse(WristConstants.kMinOutput);


    // intake motor
    m_IntakeMotor = new WPI_TalonFX(Motors.SKYHOOK_INTAKE);
    m_IntakeMotor.configFactoryDefault();
    m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    m_IntakeMotor.setSelectedSensorPosition(0);
    m_IntakeMotor.setNeutralMode(NeutralMode.Brake);
    // m_shootbottom.configClosedloopRamp(0.3);
    // m_shoottop.configClosedloopRamp(0.3);
;

    // setup PID closed-loop values
    m_IntakeMotor.config_kP(0, IntakeConstants.kP);
    m_IntakeMotor.config_kI(0, IntakeConstants.kI);
    m_IntakeMotor.config_kD(0, IntakeConstants.kD);
    m_IntakeMotor.config_IntegralZone(0, IntakeConstants.kIz);
    m_IntakeMotor.config_kF(0, IntakeConstants.kFF);
    m_IntakeMotor.configClosedLoopPeakOutput(0, IntakeConstants.kMaxOutput);
    // m_IntakeMotor.configPeakOutputForward(IntakeConstants.kMaxOutput);
    // m_IntakeMotor.configPeakOutputReverse(IntakeConstants.kMinOutput);

  }

  @Override
  public void periodic() {
       // This method will be called once per scheduler run
    // output values that show on driver screen dashboard, or are used in LED lights

    // output numbers for record-and-playback system
    SmartDashboard.putNumber("dataRecorder." + datapoint.ArmPosition, m_ArmSetpoint);
    SmartDashboard.putNumber("dataRecorder." + datapoint.ExtensionPosition, m_ExtensionSetpoint);
    SmartDashboard.putNumber("dataRecorder." + datapoint.WristPosition, m_WristSetpoint);
    SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeSpeed, m_IntakeSetpoint);

    SmartDashboard.putNumber("Arm.Position", GetArmPosition());
    SmartDashboard.putNumber("Arm Canifier", m_Canifier.getQuadraturePosition());

    //SmartDashboard.putNumber("Arm.Velocity", GetArmVelocity());
    SmartDashboard.putBoolean("ArmInSafeZone", ArmInSafeZone());
    SmartDashboard.putBoolean("ArmWantsToMove", ArmWantsToMove());
    //SmartDashboard.putNumber("Arm.Setpoint", m_ArmSetpoint);
    //SmartDashboard.putNumber("Arm.HoldSetpoint", m_ArmLastPosition);
    SmartDashboard.putNumber("Extension.Position", GetExtensionPosition());
    SmartDashboard.putNumber("Extension.Setpoint", m_ExtensionSetpoint);
    //SmartDashboard.putNumber("Extension.HoldSetpoint", m_ExtensionHoldSetpoint);
    SmartDashboard.putNumber("Wrist.Position", GetWristPosition());
    //SmartDashboard.putNumber("Wrist.Setpoint", m_WristSetpoint);
    //SmartDashboard.putNumber("Wrist.HoldSetpoint", m_WristHoldSetpoint);
    //SmartDashboard.putNumber("Intake.Setpoint", m_IntakeSetpoint);

    // only allow arm to move when extention arm is within safe extension point
    // ideally, the code will move the wrist & arm to safely pass through robot
    
    // intake motor is harmless, do whatever the driver wants
    if (m_IntakeSetpoint < 0 && powerEjectTimer < 0) {
      m_IntakeMotor.set(m_IntakeCtrlType, -1);  
    }
    else {
      powerEjectTimer --;
      m_IntakeMotor.set(m_IntakeCtrlType, m_IntakeSetpoint);
    }
    

    //always allow 0% power to Extension
    if (m_ExtensionCtrlType == ControlType.kDutyCycle && m_ExtensionSetpoint==0) {
      m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
    }

    // always allow arm to go to 0% power
    if (m_ArmCtrlType == ControlMode.PercentOutput && m_ArmSetpoint == 0) {
      m_ArmMotor.set(m_ArmCtrlType, m_ArmSetpoint);
    }
    // always allow wrist to 0% power
    if (m_WristCtrlType == ControlMode.PercentOutput && m_WristSetpoint == 0) {
      m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
    }
    
    // if arm must move, we need wrist bent and extention retracted
    String msg = "";
    if (inManualControlMode){
      // manual mode, do almost whatever the driver wants, as long as it's one at a time
      if (m_ArmSetpoint==0 && m_WristSetpoint==0){
        m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
      }
      if (m_ArmSetpoint==0 && m_ExtensionSetpoint == 0) {
        m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
      }
      if (m_ExtensionSetpoint == 0 && m_WristSetpoint == 0) {
        m_ArmMotor.set(m_ArmCtrlType, m_ArmSetpoint);
      }
    }
    else if (ArmWantsToMove()){
      msg="Arm Must Move";
      m_ExtensionPID.setReference(ExtensionPositions.RETRACTED, ControlType.kPosition);
      // if extension is safely retracted, move wrist
      if (GetExtensionPosition() >= ExtensionPositions.SAFEPOSITIONMIN) {
        msg += ", elev retracted";
        //TODO: figure out best direction to bend the wrist
        double targetPosition = WristPositions.FRONTFOLDUP;
        //if (GetArmPosition())
        m_WristMotor.set(ControlMode.Position, WristPositions.FRONTFOLDUP);
        // if wrist near desired position, then move arm!
        if (Math.abs(GetWristPosition() - targetPosition) < 3000){
          msg += ", bent OK . moving arm!";
          m_ArmMotor.set(m_ArmCtrlType, m_ArmSetpoint);
        }
        else { msg += ", wait for wrist"; }
      }
      else { msg += ", wait for extension"; }
    }

    else { // arm is where we want it, extend and bend wrist as requestd
      msg = "Arm is set, Let 'er rip, tater chip";
      //m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
      // allow extension to retract even if arm not in safe zone
      if (ArmInSafeZone() 
      || (m_ExtensionSetpoint >= ExtensionPositions.SAFESETPOINTMIN )){
        msg += ", extension set";
        m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
      }
      //m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
      // if arm is in safe zone, allow wrist to move as requested
      if (ArmInSafeZone()){ // arm is in a safe position, wrist moves as requested
          m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
          msg += ", wrist set";
      }
    }
   
    SmartDashboard.putString("ArmSafe", msg);
  }

  public void setManualControlMode(boolean _ManualControlMode){
    m_ArmCtrlType = ControlMode.PercentOutput;
    m_ArmSetpoint = 0;
    m_ExtensionCtrlType = ControlType.kDutyCycle;
    m_ExtensionSetpoint = 0;
    m_WristCtrlType = ControlMode.PercentOutput;
    m_WristSetpoint = 0;
    
    inManualControlMode = _ManualControlMode;
  }

  // methods for Arm motor
  // public void SetArmPosition(double position){
  //   if (position < ArmPositions.LOWERLIMIT) {position = ArmPositions.LOWERLIMIT;}
  //   if (position > ArmPositions.UPPERLIMIT) {position = ArmPositions.UPPERLIMIT;}

  //   m_ArmCtrlType = ControlMode.Position;
  //   m_ArmSetpoint = position;
  //  }
  //  public void SetArmVelocity(double velocity){
  //   m_ArmCtrlType = ControlMode.Velocity;
  //   m_ArmSetpoint = velocity;
  //  }
   public void SetArmSmartMotion(double position){
    if (position < ArmPositions.MINLIMIT) {position = ArmPositions.MINLIMIT;}
    if (position > ArmPositions.MAXLIMIT) {position = ArmPositions.MAXLIMIT;}

    m_ArmCtrlType = ControlMode.MotionMagic;
    m_ArmSetpoint = position;
   }
   public void SetArmPower(double percent){
    m_ArmCtrlType = ControlMode.PercentOutput;
    m_ArmSetpoint = percent;
   }
   public double GetArmPosition(){
     return m_ArmMotor.getSelectedSensorPosition();
   }
   public double GetArmVelocity(){
    return m_ArmMotor.getSelectedSensorVelocity();
   }

   public boolean ArmInSafeZone(){
    // check the arm position to determine if it in a "safe range" where the elevator and wrist can move
    
    //double position = GetArmPosition();
    // never safe if moving
    if (Math.abs(GetArmVelocity()) > (ArmConstants.kMaxAccel / 50) ) { return false; }
    return true;
    //return (GetArmPosition() < ArmPositions.UNSAFEPOSITIONMIN || GetArmPosition() > ArmPositions.UNSAFEPOSITIONMAX);
   }

   private boolean ArmWantsToMove(){
    if (m_ArmCtrlType == ControlMode.PercentOutput && m_ArmSetpoint == 0){
      return false;   // no request to move the arm
    }

    // // if arm set to go to unsafe position, assume bad things
    // if ((m_ArmSetpoint > ArmPositions.UNSAFEPOSITIONMIN)
    // && (m_ArmSetpoint < ArmPositions.UNSAFEPOSITIONMAX)) { 
    //   return true; // arm set to go to an unsafe position
    // }
    // // if arm IS in an unsafe position
    // if ((GetArmPosition() > ArmPositions.UNSAFEPOSITIONMIN)
    // && (GetArmPosition() < ArmPositions.UNSAFEPOSITIONMAX)){
    //   return true; // arm is in unsafe position
    // }

    // if arm is not near to  or more away from desired position
    if (Math.abs(GetArmPosition() - m_ArmSetpoint) > 107) {
      return true;  // arm not where requested
    }
    return false; // arm is safely where requested
   }


  //  public double GetArmHoldSetpoint(){
  //   return m_ArmLastPosition;
  //  }
   
  // methods for Extension motor
   public void SetExtensionPosition(double position){   
    if (position < ExtensionPositions.EXTENDLIMIT){ position= ExtensionPositions.EXTENDLIMIT; }
    if (position > ExtensionPositions.RETRACTLIMIT) { position = ExtensionPositions.RETRACTLIMIT; }
    m_ExtensionCtrlType = ControlType.kPosition;
    m_ExtensionSetpoint = position;
   }
  //  public void SetExtensionVelocity(double velocity){
  //   m_ExtensionCtrlType = ControlType.kVelocity;
  //   m_ExtensionSetpoint = velocity;
  //  }
  //  public void SetExtensionSmartMotion(double position){
  //   if (position < ExtensionPositions.LOWERLIMIT){ position= ExtensionPositions.LOWERLIMIT; }
  //   if (position > ExtensionPositions.UPPERLIMIT) { position = ExtensionPositions.UPPERLIMIT; }
  //   m_ExtensionCtrlType = ControlType.kSmartMotion;
  //   m_ExtensionSetpoint = position;
  //  }
   public void SetExtensionPower(double percent){
    m_ExtensionCtrlType = ControlType.kDutyCycle;
    m_ExtensionSetpoint = percent;
   }
   public double GetExtensionPosition(){
    return m_ExtensionMotor.getEncoder().getPosition();
   }
   public double GetExtensionVelocity(){
    return m_ExtensionMotor.getEncoder().getVelocity();
   }

  // methods for Wrist motor   
   public void SetWristPosition(double position){
    if (position < WristPositions.MINLIMIT){ position= WristPositions.MINLIMIT; }
    if (position > WristPositions.MAXLIMIT) { position = WristPositions.MAXLIMIT; }

    m_WristCtrlType = ControlMode.Position;// ControlType.kPosition;
    m_WristSetpoint = position;
   }
  //  public void SetWristVelocity(double velocity){
  //   m_WristCtrlType = ControlMode.Velocity;// ControlType.kVelocity;
  //   m_WristSetpoint = velocity;
  //  }
  //  public void SetWristSmartMotion(double position){
  //   m_WristCtrlType = ControlMode.MotionMagic; //ControlType.kSmartMotion;
  //   m_WristSetpoint = position;
  //  }
   public void SetWristPower(double percent){
    m_WristCtrlType = ControlMode.PercentOutput; //ControlType.kDutyCycle;
    m_WristSetpoint = percent;
   }
   public double GetWristPosition(){
    //return m_WristMotor.getEncoder().getPosition();
    return m_WristMotor.getSelectedSensorPosition();
   }
   public double GetWristVelocity(){
    //return m_WristMotor.getEncoder().getVelocity();
    return m_WristMotor.getSelectedSensorPosition();
   }

   // methods for Intake motor
   public void SetIntakePosition(double position){
    m_IntakeCtrlType = ControlMode.Position;// ControlType.kPosition;
    m_IntakeSetpoint = position; 
   }
   public void SetIntakeVelocity(double velocity){
    m_IntakeCtrlType = ControlMode.Velocity;// ControlType.kVelocity;
    m_IntakeSetpoint = velocity;
   }
   public void SetSmartMotion(double position){
    m_IntakeCtrlType = ControlMode.MotionMagic; // ControlType.kSmartMotion;
    m_IntakeSetpoint = position;
   }
   public void SetIntakePower(double percent){
    if (percent < 0){
      powerEjectTimer = 100;
    }
    else {
      powerEjectTimer = 0; //don't use
    }
    m_IntakeCtrlType = ControlMode.PercentOutput;// ControlType.kDutyCycle;
    m_IntakeSetpoint = percent;
   }
   public double GetIntakePosition(){
    //return m_IntakeMotor.getEncoder().getPosition();
    return m_IntakeMotor.getSelectedSensorPosition();
   }
   public double GetIntakeVelocity(){
    //return m_IntakeMotor.getEncoder().getVelocity();
    return m_IntakeMotor.getSelectedSensorVelocity();
   }
}
