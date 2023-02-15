/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants.Motors;
//import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DataRecorder.datapoint;

public class PancakeFlipper extends SubsystemBase {
  public static final class IntakeConstants {
    // PID values
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;
    public static final double pickupPower = .5;
    public static final double ejectPower = -.5;
  }
  
  public static final class FlipperConstants {
    // PID values
    public static final double kP = 0.4096;
    public static final double kI = 0.00;
    public static final double kD = 0;
    public static final double kIz = 8000;
    public static final double kFF = 0;//.000015;
    public static final double kMaxOutput = 0.05;
    public static final double kMinOutput = -0.05;
    public static final double homePos = 0;
    public static final double pickupPos = 1;


  }
  private final CANSparkMax m_intakeLeft;
  private final CANSparkMax m_intakeRight;
  private final CANSparkMax m_flipArm;
  private RelativeEncoder m_encoder;

  private final SparkMaxPIDController m_IntakeLeftPID, m_IntakeRightPID, m_flipArmPID;

  private double setFlipperPosition, setSpeedBottom;

  // private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;
  // private int kIz;
  private boolean manualForceReady;
  private int readyCounter;  // number of times shooters report ready in a row

  //private boolean cacheTopReady, cacheBottomReady;
  /**
   * Creates a new Shooter.
   */
  public PancakeFlipper() {
    super();

    manualForceReady = false;
    // cacheBottomReady = false;
    // cacheTopReady = false;

    readyCounter = 0;
    m_flipArm = new CANSparkMax(Motors.PancakeFlipArm, MotorType.kBrushless);
    m_flipArm.restoreFactoryDefaults();
    // reduce communication on CAN bus
    m_flipArm.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_flipArm.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_flipArm.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_flipArmPID = m_flipArm.getPIDController();
    m_flipArmPID.setP(FlipperConstants.kP);
    m_flipArmPID.setI(FlipperConstants.kI);
    m_flipArmPID.setD(FlipperConstants.kD);
    m_flipArmPID.setIZone(FlipperConstants.kIz);
    m_flipArmPID.setFF(FlipperConstants.kFF);
    m_flipArmPID.setOutputRange(FlipperConstants.kMinOutput, FlipperConstants.kMaxOutput);

    m_encoder = m_flipArm.getEncoder();
    m_encoder.setPosition(FlipperConstants.homePos);
    m_intakeLeft = new CANSparkMax(Motors.Intake_Left, MotorType.kBrushless);
    m_intakeLeft.restoreFactoryDefaults();
    // reduce communication on CAN bus
    m_intakeLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_intakeLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_intakeLeft.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_IntakeLeftPID = m_intakeLeft.getPIDController();
    m_IntakeLeftPID.setP(IntakeConstants.kP);
    m_IntakeLeftPID.setI(IntakeConstants.kI);
    m_IntakeLeftPID.setD(IntakeConstants.kD);
    m_IntakeLeftPID.setIZone(IntakeConstants.kIz);
    m_IntakeLeftPID.setFF(IntakeConstants.kFF);
    m_IntakeLeftPID.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);


    m_intakeRight = new CANSparkMax(Motors.Intake_Right, MotorType.kBrushless);
    m_intakeRight.restoreFactoryDefaults();
    // reduce communication on CAN bus
    m_intakeRight.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    m_intakeRight.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
    m_intakeRight.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);

    m_IntakeRightPID = m_intakeRight.getPIDController();
    m_IntakeRightPID.setP(IntakeConstants.kP);
    m_IntakeRightPID.setI(IntakeConstants.kI);
    m_IntakeRightPID.setD(IntakeConstants.kD);
    m_IntakeRightPID.setIZone(IntakeConstants.kIz);
    m_IntakeRightPID.setFF(IntakeConstants.kFF);
    m_IntakeRightPID.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);


    // m_shootbottom.configClosedloopRamp(0.3);
    // m_shoottop.configClosedloopRamp(0.3);
   
    setFlipperPosition = 0;
    setSpeedBottom = 0;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flipper set point", setFlipperPosition);
 }

 public void SetFlipperPos(double position){
  m_flipArmPID.setReference(position, CANSparkMax.ControlType.kPosition);
 }

 public void RunPickupMotors( double speed){
   m_intakeLeft.set(speed);
   m_intakeRight.set(-speed);
 }
 
}