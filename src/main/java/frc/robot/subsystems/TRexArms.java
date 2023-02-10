/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import edu.wpi.first.wpilibj.DigitalInput;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
 import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.DataRecorder.datapoint;


public class TRexArms extends SubsystemBase {
  private final CANSparkMax m_motor;
  // private final WPI_TalonSRX m_IntakeArm;
  private SparkMaxPIDController m_pidController;
  private boolean intakeExtended = false;
  // private double m_Intake_ArmSpeed;
  private double m_IntakeSpeed;
  private RelativeEncoder m_encoder;
  //private Solenoid m_airSolenoid;
  private  PneumaticsControlModule m_phPneumaticsControlModule;
  private Solenoid[] m_solenoids;
  private Solenoid m_airSolenoid;
  private DigitalInput m_DigitalInput;
private int counter = 0;
private PWM m_limit;


  //private DataRecorder m_dataRecorder;
 
  // public static final class IntakeArmConstants {
  //   //run arm motor to extended position, find right position
  //   public static final double armStartingPos = 0;
  //  //public static final double armRetractPos = 1000;
  //   public static final double armStopRetract = -2500;
  //   public static final double armSlowRetract = -25000;
  //   public static final double armExtendedPos = -67000;
   
    // intake arm PID values
    // public static final double kP =2; // 0.4; //0.04;
    // public static final double kI = 0; //0.0002;
    // public static final double kD = 0.0;
    // public static final double kIz = 0; //8000;
    // public static final double kFF = 8; //0.1;// 0;//.000015;
    // public static final double kMaxOutput = 0.2; // 0.3;
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700;  
//}


  public static final class MotorConstants {


    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;


    // public static final double EncoderToNumber = 1;
    // public static final double maxRPM = 5700;


  }
  /**
   * Creates a new Intake.
   */
  public TRexArms() {
    m_motor = new CANSparkMax(Motors.SAMPLE_MOTOR, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
  //  DigitalInput toplimitSwitch = new DigitalInput(2);
    // m_IntakeMotor.configClosedloopRamp(0.5);
    m_pidController = m_motor.getPIDController();
    m_pidController.setP(MotorConstants.kP);
    m_pidController.setI(MotorConstants.kI);
    m_pidController.setD(MotorConstants.kD);
    m_pidController.setIZone(MotorConstants.kIz);
    m_pidController.setFF(MotorConstants.kFF);
    m_pidController.setOutputRange(MotorConstants.kMinOutput, MotorConstants.kMaxOutput);


    m_DigitalInput = new DigitalInput(2);
    m_airSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    // m_solenoids = new Solenoid[] {new Solenoid(PneumaticsModuleType.REVPH, 0)};
      // new Solenoid(PneumaticsModuleType.REVPH, 1), new Solenoid(PneumaticsModuleType.REVPH, 2),
      // new Solenoid(PneumaticsModuleType.REVPH, 3), new Solenoid(PneumaticsModuleType.REVPH, 4),
      // new Solenoid(PneumaticsModuleType.REVPH, 5), new Solenoid(PneumaticsModuleType.REVPH, 6),
      // new Solenoid(PneumaticsModuleType.REVPH, 7), new Solenoid(PneumaticsModuleType.REVPH, 8),
      // new Solenoid(PneumaticsModuleType.REVPH, 9), new Solenoid(PneumaticsModuleType.REVPH, 10),
      // new Solenoid(PneumaticsModuleType.REVPH, 11), new Solenoid(PneumaticsModuleType.REVPH, 12),
      // new Solenoid(PneumaticsModuleType.REVPH, 13), new Solenoid(PneumaticsModuleType.REVPH, 14),
      // new Solenoid(PneumaticsModuleType.REVPH, 15)   };


     
      m_phPneumaticsControlModule = new PneumaticsControlModule();


     






    // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    // m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);


    double junk = SmartDashboard.getNumber("intakeSpeed", 35000);
    SmartDashboard.putNumber("intakeSpeed", junk);


    m_IntakeSpeed = 0;
   
    // m_IntakeArm = new WPI_TalonSRX(Motors.INTAKE_ARM);
    // m_IntakeArm.configFactoryDefault();
    // m_IntakeArm.setInverted(false);
    // m_IntakeArm.setNeutralMode(NeutralMode.Brake);
    // m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armStartingPos);
    // // PID values for INTAKE_ARM
    // m_IntakeArm.config_kP(0, IntakeArmConstants.kP);
    // m_IntakeArm.config_kI(0, IntakeArmConstants.kI);
    // m_IntakeArm.config_kD(0, IntakeArmConstants.kD);
    // m_IntakeArm.config_IntegralZone(0, IntakeArmConstants.kIz);
    // m_IntakeArm.config_kF(0, IntakeArmConstants.kFF);
    // m_IntakeArm.configClosedLoopPeakOutput(0, IntakeArmConstants.kMaxOutput);


    // m_IntakeArm.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
    // m_IntakeArm.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs)


    // m_IntakeArm.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    // m_IntakeArm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);


   // m_dataRecorder = null;
    // intakeExtended = false;
    // m_Intake_ArmSpeed = 0;
  }


  // public void setDataRecorder(DataRecorder _dataRecorder){
  //   m_dataRecorder = _dataRecorder;
  // }


   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("IntakeArmAt", m_IntakeArm.getSelectedSensorPosition());
    // SmartDashboard.putBoolean("IntakeFwdLimit", m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed());
    // SmartDashboard.putBoolean("IntakeRevLimit", m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed());


    // runMotor(m_CurrentSpeed);
        // m_motor.set(ControlMode.PercentOutput, m_CurrentSpeed);
    // m_motor.set(m_IntakeSpeed);
    m_pidController.setReference(m_IntakeSpeed, CANSparkMax.ControlType.kPosition);


    m_DigitalInput.get();
    SmartDashboard.getBoolean("m_DigitalInput", m_DigitalInput.get());
    //NetworkTableInstance.getDefault().getTable("dataRecorder").
    // if (m_encoder != null){
    //     SmartDashboard.putNumber("EncoderValue.", m_encoder.getPosition());
    // }
 
    SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeMotorSpeed, m_IntakeSpeed);
    if (intakeExtended) {
      SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 1.0);
    }
    else {
      SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 0.0);
    }
    //SmartDashboard.putBoolean("dataRecorder.extended", intakeExtended);




    //SmartDashboard.putNumber("IntakeArmAt", m_IntakeArm.getSelectedSensorPosition());
    // if upper or lower limit switch is hit, then reset encoder position to upper or lower
    // if (m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed()){
    //   m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armStartingPos);
    // }
   
    // if (intakeExtended){
    //   if (m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed()
    //       || m_IntakeArm.getSelectedSensorPosition() <= IntakeArmConstants.armExtendedPos){
    //     m_Intake_ArmSpeed = 0;
    //     stopArm();
    //   }
    //   else if (m_Intake_ArmSpeed < 0 && m_IntakeArm.getSelectedSensorPosition() <  -10000) {
    //     m_Intake_ArmSpeed = -0.55;  // speed up on lower end of extending arm
    //   }
    // }
    // else {
    //   if (m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed()
    //     || m_IntakeArm.getSelectedSensorPosition() >= IntakeArmConstants.armStopRetract ){
    //   //   m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armExtendedPos);
    //     stopArm();
    //     m_Intake_ArmSpeed = 0;
    //   }
    //   else if (m_Intake_ArmSpeed > 0 && m_IntakeArm.getSelectedSensorPosition() >= IntakeArmConstants.armSlowRetract) {
    //     m_Intake_ArmSpeed = 0.15;  // slow down as we approach closed position
    //   }
    // }
    // m_IntakeArm.set(ControlMode.PercentOutput, m_Intake_ArmSpeed);  
 
  }
  // public void runMotor(double speed){
  //   //m_IntakeMotor.set(ControlMode.PercentOutput, speed);
  //   m_IntakeMotor.set(ControlMode.Velocity, speed);
  // }
// public void extendArm(){
//   // m_IntakeArm.set(ControlMode.PercentOutput, 1);
//   SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 1.0);
//   //m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armExtendedPos);
//   intakeExtended = true;
//   m_IntakeMotor.set(ControlMode.PercentOutput, -0.3);
//   m_Intake_ArmSpeed = -0.20;// -IntakeArmConstants.kMaxOutput;
//   // if (m_dataRecorder != null) {
//   //   m_dataRecorder.recordValue(datapoint.IntakeIsExtended, (double)1.00);
//   // }
// }


// public void retractArm(){
//   //m_IntakeArm.set(ControlMode.PercentOutput, -1);
//   SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 0.0);


//   //m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armRetractPos);
//   intakeExtended = false;
//   m_Intake_ArmSpeed = 0.55;// IntakeArmConstants.kMaxOutput;
 
//   // if (m_dataRecorder != null) {
//   //   m_dataRecorder.recordValue(datapoint.IntakeIsExtended, (double)0.00);
//   // }


// }


// public void stopArm() {
//   m_IntakeArm.set(ControlMode.PercentOutput, 0);
//   m_Intake_ArmSpeed = 0;
// }


public void runIntake(double position){
  m_IntakeSpeed = position;
  // m_motor.set(.025);
  // if (m_DigitalInput.get()) {
    // m_motor.set(0);
  // }
  }


  public void HeimlichManeuver() {
    runIntake( -1 * SmartDashboard.getNumber("intakeSpeed", 25000));
  }
  public void StopIntake() {
    runIntake(0);
    m_phPneumaticsControlModule.disableCompressor();
    m_airSolenoid.set(false);
    // for (Solenoid solenoid : m_solenoids) {
    //   solenoid.set(false);
    // }
  }
  public void StartIntake() {
    runIntake(SmartDashboard.getNumber("intakeSpeed", 1));
    m_phPneumaticsControlModule.enableCompressorDigital();
    m_airSolenoid.set(true);


    // counter ++;
// SmartDashboard.putNumber("Counter", counter);


    // int i = 0;
    // boolean setOn = false;
    // for (Solenoid solenoid : m_solenoids) {
    //   setOn = (counter & i) == i;
    //   solenoid.set(setOn);
    //   i ++;
    // }


  }


  public void ZeroEncoder(){
    m_encoder = m_motor.getEncoder();
    SmartDashboard.putNumber("EncoderValue.", m_encoder.getPosition());
    // m_encoder.setPosition(0);
    // SmartDashboard.putNumber("EncoderValueZero.", m_encoder.getPosition());
  }
  public void ArmExtend(){
runIntake(.5);
SmartDashboard.putNumber("EncoderValue.", m_encoder.getPosition());
  }
  public void ArmRetract(){
runIntake(1);
SmartDashboard.putNumber("EncoderValue.", m_encoder.getPosition());
  }
  // public void allowAdditionalMovement(){
  //   m_IntakeArm.setSelectedSensorPosition( IntakeArmConstants.armExtendedPos / 2);
  // }
 
  }
 

