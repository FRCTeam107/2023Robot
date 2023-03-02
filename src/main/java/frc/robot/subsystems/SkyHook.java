/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

public class SkyHook extends SubsystemBase {

  private final CANSparkMax m_ExtensionMotor, m_ArmMotor;
  private final WPI_TalonFX m_WristMotor, m_IntakeMotor;
  //ControlMode.Velocity
  private ControlMode m_WristCtrlType, m_IntakeCtrlType;

  
  private final SparkMaxPIDController m_ExtensionPID, m_ArmPID;//, m_WristPID, m_IntakePID;

  private CANSparkMax.ControlType m_ExtensionCtrlType, m_ArmCtrlType;
  private double m_ExtensionSetpoint, m_ArmSetpoint, m_WristSetpoint, m_IntakeSetpoint;
  private double m_ArmHoldSetpoint, m_ExtensionHoldSetpoint, m_WristHoldSetpoint;

  public static final class ArmPositions{
    static final double UPPERLIMIT = 20; // maximum value for position
    public static final double BACK = 19;
    public static final double FORWARD = -12;
    public static final double STARTPOSITION = 0;
    static final double LOWERLIMIT = -20; // minimum value for position
    }
  public static final class ExtensionPositions{
    static final double UPPERLIMIT = 0; // actual limit (upper limit switch hit)
    public static final double RETRACTED = -10; // advertised retracted position
    static final double STARTPOSITION = -124;
    public static final double EXTENDED = -200;
    static final double LOWERLIMIT = -220; // fully extended position
  }

static final class ExtensionConstants {
    // PID values
    static final double kP = 0.04;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 80;
    static final double kFF = 0.01;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
  }

  static final class ArmConstants { 
    // PID values
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = 0.01;//.000015;
    static final double kMaxOutput = 0.4;
    static final double kMinOutput = -0.4;
  }
  static final class WristConstants { 
    // PID values
    static final double kP = 0.03;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = 0.08;//.000015;
    static final double kMaxOutput = 0.5;
    static final double kMinOutput = -0.5;
  }
  static final class IntakeConstants { 
    // PID values
    static final double kP = 0.4096;
    static final double kI = 0.00;
    static final double kD = 0;
    static final double kIz = 8000;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 1;
    static final double kMinOutput = -1;
  }
/**
   * Creates a new SkyHook.
   */
  public SkyHook() {
    super();

    m_ExtensionCtrlType = ControlType.kDutyCycle;
    m_ArmCtrlType = ControlType.kDutyCycle;
    m_WristCtrlType = ControlMode.PercentOutput;// ControlType.kDutyCycle;
    m_IntakeCtrlType = ControlMode.PercentOutput;//ControlType.kDutyCycle;
    m_ExtensionSetpoint = 0;
    m_ArmSetpoint = 0;
    m_WristSetpoint = 0;
  
    m_ExtensionMotor = new CANSparkMax(Motors.SKYHOOK_EXTENDER, MotorType.kBrushless);
    m_ExtensionMotor.restoreFactoryDefaults();
    m_ExtensionMotor.setIdleMode(IdleMode.kBrake);
    m_ExtensionMotor.getEncoder().setPosition(ExtensionPositions.STARTPOSITION);

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
    m_ArmMotor.getEncoder().setPosition(ArmPositions.STARTPOSITION);

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
    m_ArmMotor.burnFlash();

    // Wrist to bend the intake head
    m_WristMotor = new WPI_TalonFX(Motors.SKYHOOK_WRIST);
    m_WristMotor.configFactoryDefault();
    m_WristMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    m_WristMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    m_WristMotor.setSelectedSensorPosition(0);
    m_WristMotor.setNeutralMode(NeutralMode.Brake);
    // m_shootbottom.configClosedloopRamp(0.3);
    // m_shoottop.configClosedloopRamp(0.3);
;

    // setup PID closed-loop values
    // kP = 0.25; 
    // kI = 0.0005;
    // kD = 0.0001; 
    // kIz = 8000; 
    // kFF = 0;//.000015; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;
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
    // kP = 0.25; 
    // kI = 0.0005;
    // kD = 0.0001; 
    // kIz = 8000; 
    // kFF = 0;//.000015; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;
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
    if (m_ArmCtrlType == ControlType.kPosition || m_ArmCtrlType==ControlType.kSmartMotion){
      m_ArmHoldSetpoint = m_ArmSetpoint;
    }
    else {
      m_ArmHoldSetpoint = GetArmPosition();
    }
    if (m_ExtensionCtrlType == ControlType.kPosition || m_ExtensionCtrlType==ControlType.kSmartMotion){
      m_ExtensionHoldSetpoint = m_ExtensionSetpoint;
    }
    // else {
    //   m_ExtensionHoldSetpoint = GetExtensionPosition();
    // }
    if (m_WristCtrlType == ControlMode.Position || m_WristCtrlType==ControlMode.MotionMagic ){
      m_WristHoldSetpoint = m_WristSetpoint;
    }
    // else {
    //   m_WristHoldSetpoint = GetWristPosition();
    // }    
    // This method will be called once per scheduler run
    // output values that show on driver screen dashboard, or are used in LED lights
    SmartDashboard.putNumber("Arm.Position", GetArmPosition());
    SmartDashboard.putNumber("Arm.Setpoint", m_ArmSetpoint);
    SmartDashboard.putNumber("Arm.HoldSetpoint", m_ArmHoldSetpoint);
    SmartDashboard.putNumber("Extension.Position", GetExtensionPosition());
    SmartDashboard.putNumber("Extension.Setpoint", m_ExtensionSetpoint);
    SmartDashboard.putNumber("Extension.HoldSetpoint", m_ExtensionHoldSetpoint);
    SmartDashboard.putNumber("Wrist.Position", GetWristPosition());
    SmartDashboard.putNumber("Wrist.Setpoint", m_WristSetpoint);
    SmartDashboard.putNumber("Wrist.HoldSetpoint", m_WristHoldSetpoint);
    //SmartDashboard.putNumber("Intake.Setpoint", m_IntakeSetpoint);

    // TODO: put code here to prevent moving arm through robot if extension or wrist in unsafe position
    // ideally, the code will move the wrist & arm to safely pass through robot

    SmartDashboard.putNumber("Arm to", m_ArmSetpoint);
    m_ArmPID.setReference(m_ArmSetpoint, m_ArmCtrlType);
    m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
    m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
    m_IntakeMotor.set(m_IntakeCtrlType, m_IntakeSetpoint);
    //m_IntakeMotor.set(ControlMode.PercentOutput, m_IntakeSetpoint)
    //m_IntakePID.setReference(m_IntakeSetpoint, m_IntakeCtrlType);
  }

  // methods for Arm motor
  public void SetArmPosition(double position){
    if (position < ArmPositions.LOWERLIMIT) {position = ArmPositions.LOWERLIMIT;}
    if (position > ArmPositions.UPPERLIMIT) {position = ArmPositions.UPPERLIMIT;}

    m_ArmCtrlType = ControlType.kPosition;
    m_ArmSetpoint = position;
   }
   public void SetArmVelocity(double velocity){
    m_ArmCtrlType = ControlType.kVelocity;
    m_ArmSetpoint = velocity;
   }
   public void SetArmSmartMotion(double position){
    if (position < ArmPositions.LOWERLIMIT) {position = ArmPositions.LOWERLIMIT;}
    if (position > ArmPositions.UPPERLIMIT) {position = ArmPositions.UPPERLIMIT;}

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
   public double GetArmHoldSetpoint(){
    return m_ArmHoldSetpoint;
   }
   
  // methods for Extension motor
   public void SetExtensionPosition(double position){
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
