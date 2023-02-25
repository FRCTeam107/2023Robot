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

  public static final class ArmFlip{
    public static final double BACK = 29;
    public static final double FORWARD = -12;
    public static final double HOME = 0;
    public static final double RETRACTED = 0;
    public static final double EXTENDED = 10;
    }


  static final class ExtensionConstants {
    // PID values
    static final double kP = 5e-5;
    static final double kI = 1e-6;
    static final double kD = 0;
<<<<<<< HEAD
    static final double kIz = 0;
    static final double kFF = .000156;
    static final double kMaxOutput = 10;
    static final double kMinOutput = -1;
    static final double kMaxAccel = 1500;
    static final double kMaxVel = 2000;
    static final double MaxPRM = 5700;
    }
=======
    static final double kIz = 8000;
    static final double kFF = 0;//.000015;
    static final double kMaxOutput = 0.05;
    static final double kMinOutput = -0.05;
  }
>>>>>>> c5108bc7c289b26b9d9c54cddb93ff45b2b9f6c5

  static final class ArmConstants { 
    // PID values
    static final double kP = 5e-5;
    static final double kI = 1e-6;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = .000156;
    static final double kMaxOutput = 10;
    static final double kMinOutput = -1;
    static final double kMaxAccel = 1500;
    static final double kMaxVel = 2000;
    static final double MaxPRM = 5700;
  }
  static final class WristConstants { 
    // PID values
    static final double kP = 5e-5;
    static final double kI = 1e-6;
    static final double kD = 0;
    static final double kIz = 0;
    static final double kFF = .000156;
    static final double kMaxOutput = 10;
    static final double kMinOutput = -1;
    static final double kMaxAccel = 1500;
    static final double kMaxVel = 2000;
    static final double MaxPRM = 5700;
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
    m_WristCtrlType = ControlMode.PercentOutput;// ControlType.kDutyCycle;
    m_IntakeCtrlType = ControlMode.PercentOutput;//ControlType.kDutyCycle;
    m_ExtensionSetpoint = 0;
    m_ArmSetpoint = 0;
    m_WristSetpoint = 0;
  
    m_ExtensionMotor = new CANSparkMax(Motors.SKYHOOK_EXTENDER, MotorType.kBrushless);
    m_ExtensionMotor.restoreFactoryDefaults();
    m_ExtensionMotor.setIdleMode(IdleMode.kBrake);
    m_ExtensionMotor.getEncoder().setPosition(0);

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
    m_ExtensionPID.setSmartMotionMaxAccel(ExtensionConstants.kMaxAccel, 0);
    m_ExtensionPID.setSmartMotionMaxVelocity(ExtensionConstants.kMaxVel, 0);
    

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
    m_ArmPID.setSmartMotionMaxAccel(ArmConstants.kMaxAccel, 0);
    m_ArmPID.setSmartMotionMaxVelocity(ArmConstants.kMaxVel, 0);

    // Wrist to bend the intake head
<<<<<<< HEAD
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
    m_WristPID.setSmartMotionMaxAccel(WristConstants.kMaxAccel,0);
    m_WristPID.setSmartMotionMaxVelocity(WristConstants.kMaxVel, 0);
=======
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


    // m_WristMotor = new CANSparkMax(Motors.SKYHOOK_WRIST, MotorType.kBrushless);
    // m_WristMotor.restoreFactoryDefaults();
    // m_WristMotor.setIdleMode(IdleMode.kBrake);
    // m_WristMotor.getEncoder().setPosition(0);

    // // reduce communication on CAN bus
    // m_WristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // m_WristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    // m_WristMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    // m_WristPID = m_WristMotor.getPIDController();
    // m_WristPID.setP(WristConstants.kP);
    // m_WristPID.setI(WristConstants.kI);
    // m_WristPID.setD(WristConstants.kD);
    // m_WristPID.setIZone(WristConstants.kIz);
    // m_WristPID.setFF(WristConstants.kFF);
    // m_WristPID.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
>>>>>>> c5108bc7c289b26b9d9c54cddb93ff45b2b9f6c5

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

    // m_IntakeMotor = new CANSparkMax(Motors.SKYHOOK_INTAKE, MotorType.kBrushless);
    // m_IntakeMotor.restoreFactoryDefaults();
    // m_IntakeMotor.setIdleMode(IdleMode.kBrake);
    // m_IntakeMotor.getEncoder().setPosition(0);
    // // reduce communication on CAN bus
    // m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    // m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    // m_IntakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    // m_IntakePID = m_IntakeMotor.getPIDController();
    // m_IntakePID.setP(IntakeConstants.kP);
    // m_IntakePID.setI(IntakeConstants.kI);
    // m_IntakePID.setD(IntakeConstants.kD);
    // m_IntakePID.setIZone(IntakeConstants.kIz);
    // m_IntakePID.setFF(IntakeConstants.kFF);
    // m_IntakePID.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // output values that show on driver screen dashboard, or are used in LED lights
    SmartDashboard.putNumber("SkyHookArm", GetArmPosition());
    SmartDashboard.putNumber("SkyHookExtension", GetExtensionPosition());
    SmartDashboard.putNumber("SkyHookWrist", GetWristPosition());
<<<<<<< HEAD
    SmartDashboard.putNumber("SkyHookExtension", GetExtensionPosition());
=======
    SmartDashboard.putNumber("SkyHookIntake", m_IntakeSetpoint);
    //SmartDashboard.putNumber("SkyHookExtension", GetExtensionPosition());
>>>>>>> c5108bc7c289b26b9d9c54cddb93ff45b2b9f6c5

    // TODO: put code here to prevent moving arm through robot if extension or wrist in unsafe position
    // ideally, the code will move the wrist & arm to safely pass through robot

    m_ArmPID.setReference(m_ArmSetpoint, m_ArmCtrlType);
    //m_ExtensionPID.setReference(m_ExtensionSetpoint, m_ExtensionCtrlType);
    m_WristMotor.set(m_WristCtrlType, m_WristSetpoint);
    //m_WristPID.setReference(m_WristSetpoint, m_WristCtrlType);
    m_IntakeMotor.set(m_IntakeCtrlType, m_IntakeSetpoint);
    //m_IntakeMotor.set(ControlMode.PercentOutput, m_IntakeSetpoint)
    //m_IntakePID.setReference(m_IntakeSetpoint, m_IntakeCtrlType);
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
    SmartDashboard.putNumber("WristPower", m_WristSetpoint);
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
<<<<<<< HEAD
   public void SetIntakeSmartMotion(double position){
    m_IntakeCtrlType = ControlType.kSmartMotion;
=======
   public void SetSmartMotion(double position){
    m_IntakeCtrlType = ControlMode.MotionMagic; // ControlType.kSmartMotion;
>>>>>>> c5108bc7c289b26b9d9c54cddb93ff45b2b9f6c5
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
