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

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.DataRecorder.datapoint;

public class TRexArms extends SubsystemBase {
private final CANSparkMax m_leftUpDown,m_rightUpDown;
private final CANSparkMax m_leftElbow, m_rightElbow;
private final CANSparkMax m_leftFingertips, m_rightFingertips;
private SparkMaxPIDController m_leftUpDown_pidController;
private SparkMaxPIDController m_rightUpDown_pidController;
private SparkMaxPIDController m_leftElbow_pidController;
private SparkMaxPIDController m_rightElbow_pidController;
private SparkMaxPIDController m_leftFingertips_pidController;
private SparkMaxPIDController m_rightFingertips_pidController;
private double m_leftFingertipsSpeed,m_rightFingertipsSpeed;
private RelativeEncoder m_encoder;

  // private final WPI_TalonSRX m_IntakeArm;
  private SparkMaxPIDController m_pidController;
  private boolean intakeExtended = false;
  // private double m_Intake_ArmSpeed;
  private double m_IntakeSpeed;
  // private RelativeEncoder m_encoder;
  //private Solenoid m_airSolenoid;
  private DigitalInput m_DigitalInput;
private int counter = 0;
private PWM m_limit;


  //private DataRecorder m_dataRecorder;
 
  public static final class UpDownMotorConstants {
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;
  }

  public static final class ElbowMotorConstants {
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;
  }

  public static final class FingertipsMotorConstants {
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;
  }
  /**
   * Creates a new T-Rex.
   */
  public TRexArms() {
    m_leftUpDown = new CANSparkMax(Motors.LEFT_UP_DOWN_MOTOR, MotorType.kBrushless);
    m_rightUpDown = new CANSparkMax(Motors.RIGHT_UP_DOWN_MOTOR, MotorType.kBrushless);
    m_leftElbow = new CANSparkMax(Motors.LEFT_ELBOW_MOTOR,MotorType.kBrushless);
    m_rightElbow = new CANSparkMax(Motors.RIGHT_ELBOW_MOTOR,MotorType.kBrushless);
    m_leftFingertips = new CANSparkMax(Motors.LEFT_FINGERTIPS_MOTOR,MotorType.kBrushless);
    m_rightFingertips = new CANSparkMax(Motors.RIGHT_FINGERTIPS_MOTOR,MotorType.kBrushless);

    //Set factory defaults
    m_leftUpDown.restoreFactoryDefaults();
    m_rightUpDown.restoreFactoryDefaults();
    m_leftElbow.restoreFactoryDefaults()  ;
    m_rightElbow.restoreFactoryDefaults();
    m_leftFingertips.restoreFactoryDefaults();
    m_leftFingertips.restoreFactoryDefaults();
    
   //
    m_leftUpDown_pidController = m_leftUpDown.getPIDController();
    m_leftUpDown_pidController.setP(UpDownMotorConstants.kP);
    m_leftUpDown_pidController.setI(UpDownMotorConstants.kI);
    m_leftUpDown_pidController.setD(UpDownMotorConstants.kD);
    m_leftUpDown_pidController.setIZone(UpDownMotorConstants.kIz);
    m_leftUpDown_pidController.setFF(UpDownMotorConstants.kFF);
    m_leftUpDown_pidController.setOutputRange(UpDownMotorConstants.kMinOutput, UpDownMotorConstants.kMaxOutput);

    m_rightUpDown_pidController = m_rightUpDown.getPIDController();
    m_rightUpDown_pidController.setP(UpDownMotorConstants.kP);
    m_rightUpDown_pidController.setI(UpDownMotorConstants.kI);
    m_rightUpDown_pidController.setD(UpDownMotorConstants.kD);
    m_rightUpDown_pidController.setIZone(UpDownMotorConstants.kIz);
    m_rightUpDown_pidController.setFF(UpDownMotorConstants.kFF);
    m_rightUpDown_pidController.setOutputRange(UpDownMotorConstants.kMinOutput, UpDownMotorConstants.kMaxOutput);

    m_leftElbow_pidController = m_leftElbow.getPIDController();
    m_leftElbow_pidController.setP(ElbowMotorConstants.kP);
    m_leftElbow_pidController.setI(ElbowMotorConstants.kI);
    m_leftElbow_pidController.setD(ElbowMotorConstants.kD);
    m_leftElbow_pidController.setIZone(ElbowMotorConstants.kIz);
    m_leftElbow_pidController.setFF(ElbowMotorConstants.kFF);
    m_leftElbow_pidController.setOutputRange(ElbowMotorConstants.kMinOutput, ElbowMotorConstants.kMaxOutput);

    m_rightElbow_pidController = m_rightElbow.getPIDController();
    m_rightElbow_pidController.setP(ElbowMotorConstants.kP);
    m_rightElbow_pidController.setI(ElbowMotorConstants.kI);
    m_rightElbow_pidController.setD(ElbowMotorConstants.kD);
    m_rightElbow_pidController.setIZone(ElbowMotorConstants.kIz);
    m_rightElbow_pidController.setFF(ElbowMotorConstants.kFF);
    m_rightElbow_pidController.setOutputRange(ElbowMotorConstants.kMinOutput, ElbowMotorConstants.kMaxOutput);

    m_leftFingertips_pidController = m_leftFingertips.getPIDController();
    m_leftFingertips_pidController.setP( FingertipsMotorConstants.kP);
    m_leftFingertips_pidController.setI( FingertipsMotorConstants.kI);
    m_leftFingertips_pidController.setD( FingertipsMotorConstants.kD);
    m_leftFingertips_pidController.setIZone( FingertipsMotorConstants.kIz);
    m_leftFingertips_pidController.setFF( FingertipsMotorConstants.kFF);
    m_leftFingertips_pidController.setOutputRange( FingertipsMotorConstants.kMinOutput,  FingertipsMotorConstants.kMaxOutput);
    
    m_rightFingertips_pidController = m_rightFingertips.getPIDController();
    m_rightFingertips_pidController.setP( FingertipsMotorConstants.kP);
    m_rightFingertips_pidController.setI( FingertipsMotorConstants.kI);
    m_rightFingertips_pidController.setD( FingertipsMotorConstants.kD);
    m_rightFingertips_pidController.setIZone( FingertipsMotorConstants.kIz);
    m_rightFingertips_pidController.setFF( FingertipsMotorConstants.kFF);
    m_rightFingertips_pidController.setOutputRange( FingertipsMotorConstants.kMinOutput,  FingertipsMotorConstants.kMaxOutput);


    double junk = SmartDashboard.getNumber("intakeSpeed", 35000);
    SmartDashboard.putNumber("intakeSpeed", junk);

    m_leftFingertipsSpeed=0;
    m_rightFingertipsSpeed=0;
   
  }

  public void something(){
    SmartDashboard.putString("Flipper", "I am working");
  }


  @Override
  public void periodic() {
    m_leftFingertips_pidController.setReference(m_leftFingertipsSpeed, CANSparkMax.ControlType.kVelocity);    
    m_rightFingertips_pidController.setReference(m_rightFingertipsSpeed, CANSparkMax.ControlType.kVelocity);
  }


  public void runSlapper (double leftPosition, double rightPosition){
    m_leftUpDown_pidController.setReference(leftPosition, CANSparkMax.ControlType.kPosition);
    m_rightUpDown_pidController.setReference(rightPosition, CANSparkMax.ControlType.kPosition);
  
  }

  public void runClapper (double leftPosition, double rightPosition){
    m_leftElbow_pidController.setReference(leftPosition, CANSparkMax.ControlType.kPosition);
    m_rightElbow_pidController.setReference(rightPosition, CANSparkMax.ControlType.kPosition);
  }

  public void runTapper (double speed){
    m_leftFingertips.set(speed);
    m_rightFingertips.follow(m_leftFingertips, true);
  }
}
