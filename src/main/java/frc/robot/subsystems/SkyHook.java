/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.DataRecorder.datapoint;

public class SkyHook extends SubsystemBase {

  private final CANSparkMax m_ExtensionMotor;
  private final WPI_TalonFX m_WristMotor, m_IntakeMotor, m_ArmMotor, m_ArmMotor2;
  private ControlMode m_WristCtrlType, m_IntakeCtrlType, m_ArmCtrlType;

  
  private final SparkMaxPIDController m_ExtensionPID;
  private CANSparkMax.ControlType m_ExtensionCtrlType;

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
      static final double UPPERLIMIT = 135000; // maximum value for position
      public static final double BACK = 117000;
      static final double UNSAFEPOSITIONMAX = 1000; // upper point where not safe to extend elevator, and wrist must fold up
      public static final double STARTPOSITION = 0;
      public static final double GROUNDPICKUP = 2000;
      static final double UNSAFEPOSITIONMIN= -30000; // lower point where not safe to extend elevator, and wrist must fold up
      public static final double FORWARD = -45000;
      static final double LOWERLIMIT = -50000; // minimum value for position
      }
  public static final class ExtensionPositions{
    static final double UPPERLIMIT = 53;  //0; // actual limit (upper limit switch hit)
    public static final double RETRACTED = 50;//-5; // advertised retracted position
    //static final double SAFELYRETRACTEDMIN = 55;//-10; // safe enough to pass through robot
    static final double STARTPOSITION = 0;//-85;
    public static final double GROUNDPICKUP = 13;
    public static final double EXTENDED = -91;//-100;
    static final double LOWERLIMIT = -98; // fully extended position

    //encoder position doesn't match setpoint for some reason
    static final double SAFESETPOINTMIN = 48;
    static final double SAFEPOSITIONMIN = 63; //setpoint=48
  }
  public static final class WristPositions{
    static final double UPPERLIMIT = 0; //9000; // actual limit (upper limit switch hit)
    public static final double RETRACTED = 2;//-8600; // advertised retracted position
    static final double STARTPOSITION = 0;//-9000;//-13000;
    public static final double GROUNDPICKUP = 300;
    public static final double EXTENDED = 17000;//8600;
    static final double LOWERLIMIT = 0;//-9000; // fully extended position
  }
static final class ExtensionConstants {
    // PID values
    static final double kP = 0.03;
    static final double kI = 0.00001;
    static final double kD = 0;
    static final double kIz = 5;
    static final double kFF = 0.01;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
  }

  static final class ArmConstants { 
    // PID values
    static final double kP = 0.2;//0.03;
    static final double kI = 1e-6;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = 0.05;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
    static final double kCruiseVelocity = 16000;
    static final double kMaxAccel = 8000;
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
    double var;
    var = SmartDashboard.getNumber("Arm To", 0.0);
    SmartDashboard.putNumber("Arm To", var);
    var = SmartDashboard.getNumber("Wrist To", 0.0);
    SmartDashboard.putNumber("Wrist To", var);
    // var = SmartDashboard.getNumber("Extension To", 0.0);
    // SmartDashboard.putNumber("Extension To", var);

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
    m_ArmMotor = new WPI_TalonFX(Motors.SKYHOOK_ARM);
    m_ArmMotor.configFactoryDefault();
    m_ArmMotor.config_kP(0, ArmConstants.kP);
    m_ArmMotor.config_kI(0, ArmConstants.kI);
    m_ArmMotor.config_kD(0, ArmConstants.kD);
    m_ArmMotor.config_IntegralZone(0, ArmConstants.kIz);
    m_ArmMotor.config_kF(0, ArmConstants.kFF);
    m_ArmMotor.configClosedLoopPeakOutput(0, ArmConstants.kMaxOutput);
    m_ArmMotor.configPeakOutputForward(ArmConstants.kMaxOutput);
    m_ArmMotor.configPeakOutputReverse(ArmConstants.kMinOutput);
    m_ArmMotor.configMotionAcceleration(ArmConstants.kMaxAccel);
    m_ArmMotor.configMotionCruiseVelocity(ArmConstants.kCruiseVelocity);
   // m_ArmMotor.configuration

    m_ArmMotor2 = new WPI_TalonFX(Motors.SKYHOOK_ARM2);
    m_ArmMotor2.configFactoryDefault();
    m_ArmMotor2.follow(m_ArmMotor);
    m_ArmMotor2.setInverted(false);
    // m_ArmMotor = new CANSparkMax(Motors.SKYHOOK_RIGHTARM, MotorType.kBrushless);
    // m_ArmMotor.restoreFactoryDefaults();
    // m_ArmMotor.setIdleMode(IdleMode.kBrake);
    // m_ArmMotor.getEncoder().setPosition(ArmPositions.STARTPOSITION);
    // m_ArmMotor.setSmartCurrentLimit(20);

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
    m_WristMotor.setSelectedSensorPosition(WristPositions.STARTPOSITION * 4);
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

    // if (m_ArmCtrlType == ControlMode.Position || m_ArmCtrlType==ControlMode.MotionMagic){
    //   m_ArmLastPosition = m_ArmSetpoint;
    // }
    // else {
    //   m_ArmLastPosition = GetArmPosition();
    // }
    // if (m_ExtensionCtrlType == ControlType.kPosition || m_ExtensionCtrlType==ControlType.kSmartMotion){
    //   m_ExtensionHoldSetpoint = m_ExtensionSetpoint;
    // }
    // // else {
    // //   m_ExtensionHoldSetpoint = GetExtensionPosition();
    // // }
    // if (m_WristCtrlType == ControlMode.Position || m_WristCtrlType==ControlMode.MotionMagic ){
    //   m_WristHoldSetpoint = m_WristSetpoint;
    // }
    // else {
    //   m_WristHoldSetpoint = GetWristPosition();
    // }    
 

    SmartDashboard.putNumber("Arm.Position", GetArmPosition());
    SmartDashboard.putBoolean("ArmInSafeZone", ArmInSafeZone());
    //SmartDashboard.putNumber("Arm.Setpoint", m_ArmSetpoint);
    //SmartDashboard.putNumber("Arm.HoldSetpoint", m_ArmLastPosition);
    SmartDashboard.putNumber("Extension.Position", GetExtensionPosition());
    SmartDashboard.putNumber("Extension.Setpoint", m_ExtensionSetpoint);
    //SmartDashboard.putNumber("Extension.HoldSetpoint", m_ExtensionHoldSetpoint);
    SmartDashboard.putNumber("Wrist.Position", GetWristPosition());
    //SmartDashboard.putNumber("Wrist.Setpoint", m_WristSetpoint);
    //SmartDashboard.putNumber("Wrist.HoldSetpoint", m_WristHoldSetpoint);
    //SmartDashboard.putNumber("Intake.Setpoint", m_IntakeSetpoint);

    // TODO: safety code  to prevent moving arm through robot if extension or wrist in unsafe position

    // only allow arm to move when extention arm is within safe extension point
    // ideally, the code will move the wrist & arm to safely pass through robot
    
    // intake motor is harmless, do whatever the driver wants
    m_IntakeMotor.set(m_IntakeCtrlType, m_IntakeSetpoint);
    
    // always allow arm to go to 0% power
    if (m_ArmCtrlType == ControlMode.PercentOutput && m_ExtensionSetpoint==0) {
      m_ArmMotor.set(m_ArmCtrlType, m_ArmSetpoint);
    }
    else {
      // only allow arm to move when extension is retracted to safe point of retraction
      // and setpoint in safe range or set to 0 (don't move)
      if (GetExtensionPosition() >= ExtensionPositions.SAFEPOSITIONMIN 
      && ( m_ExtensionSetpoint >= ExtensionPositions.SAFESETPOINTMIN
            || m_ExtensionSetpoint == 0) ){
        m_ArmMotor.set(m_ArmCtrlType, m_ArmSetpoint);
      }
    }

    // if arm is in safe zone, allow wrist and extension to move as requested
    if (ArmInSafeZone()){ // arm is in a safe position, extension and wrist move as requested
      m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
      m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
    }
    else {
      //always allow 0% power to Extension
      if (m_ExtensionCtrlType == ControlType.kDutyCycle && m_ExtensionSetpoint==0) {
        m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
      }
      else { // do special things to keep extension and wrist from crashing
        // extension arm may only retract if arm is in unsafe zone
        if (m_ExtensionSetpoint >= ExtensionPositions.SAFESETPOINTMIN ){
          SmartDashboard.putNumber("Unsafe Ext Move", m_ExtensionSetpoint);
          m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
        }
        else {
          SmartDashboard.putNumber("Unsafe Ext Move", -999.0);
        }
      }

      //always allow 0% power to Wrist
      if (m_WristCtrlType == ControlMode.PercentOutput && m_WristSetpoint==0) {
        m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
      }

    }

      
      
      //TODO, figure out where it is safest to put wrist 
      // especially important for robot in starting configuration
      // if (GetWristPosition() < WristPositions.STARTPOSITION) {
      //    m_WristMotor.set(ControlMode.Position, WristPositions.RETRACTED);
      //  }
      // else {
      //   m_WristMotor.set(ControlMode.Position, WristPositions.EXTENDED);
      // }
  
    

    //SmartDashboard.getNumber("Arm to", m_ArmSetpoint);
   //m_ArmPID.setReference(m_ArmSetpoint, m_ArmCtrlType);
    

    //m_IntakeMotor.set(ControlMode.PercentOutput, m_IntakeSetpoint)
    //m_IntakePID.setReference(m_IntakeSetpoint, m_IntakeCtrlType);
  }

  // methods for Arm motor
  public void SetArmPosition(double position){
    if (position < ArmPositions.LOWERLIMIT) {position = ArmPositions.LOWERLIMIT;}
    if (position > ArmPositions.UPPERLIMIT) {position = ArmPositions.UPPERLIMIT;}

    m_ArmCtrlType = ControlMode.Position;
    m_ArmSetpoint = position;
   }
   public void SetArmVelocity(double velocity){
    m_ArmCtrlType = ControlMode.Velocity;
    m_ArmSetpoint = velocity;
   }
   public void SetArmSmartMotion(double position){

    //position = SmartDashboard.getNumber("Arm To", position);
    
    if (position < ArmPositions.LOWERLIMIT) {position = ArmPositions.LOWERLIMIT;}
    if (position > ArmPositions.UPPERLIMIT) {position = ArmPositions.UPPERLIMIT;}

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
    if (GetArmVelocity() > 0.01) { return false; }

    return (GetArmPosition() < ArmPositions.UNSAFEPOSITIONMIN || GetArmPosition() > ArmPositions.UNSAFEPOSITIONMAX);
   }
  //  public double GetArmHoldSetpoint(){
  //   return m_ArmLastPosition;
  //  }
   
  // methods for Extension motor
   public void SetExtensionPosition(double position){
    //position = SmartDashboard.getNumber("Extension To", position);
    
    if (position < ExtensionPositions.LOWERLIMIT){ position= ExtensionPositions.LOWERLIMIT; }
    if (position > ExtensionPositions.UPPERLIMIT) { position = ExtensionPositions.UPPERLIMIT; }
    m_ExtensionCtrlType = ControlType.kPosition;
    m_ExtensionSetpoint = position;
   }
   public void SetExtensionVelocity(double velocity){
    m_ExtensionCtrlType = ControlType.kVelocity;
    m_ExtensionSetpoint = velocity;
   }
   public void SetExtensionSmartMotion(double position){
    if (position < ExtensionPositions.LOWERLIMIT){ position= ExtensionPositions.LOWERLIMIT; }
    if (position > ExtensionPositions.UPPERLIMIT) { position = ExtensionPositions.UPPERLIMIT; }
    m_ExtensionCtrlType = ControlType.kSmartMotion;
    m_ExtensionSetpoint = position;
   }
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
    position = SmartDashboard.getNumber("Wrist To", position);
    m_WristCtrlType = ControlMode.Position;// ControlType.kPosition;
    m_WristSetpoint = position;
   }
   public void SetWristVelocity(double velocity){
    m_WristCtrlType = ControlMode.Velocity;// ControlType.kVelocity;
    m_WristSetpoint = velocity;
   }
   public void SetWristSmartMotion(double position){
    m_WristCtrlType = ControlMode.MotionMagic; //ControlType.kSmartMotion;
    m_WristSetpoint = position;
   }
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
