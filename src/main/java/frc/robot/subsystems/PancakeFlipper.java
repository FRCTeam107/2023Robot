/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
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
  }
  private final CANSparkMax m_intakeLeft, m_intakeRight;
  private final CANSparkMax m_flipArm;

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
    m_flipArmPID.setReference(setFlipperPosition, CANSparkMax.ControlType.kPosition);
 }

 public void FlipUp(){
 setFlipperPosition = .5;
 }
 public void FlipDown(){
  setFlipperPosition = 0;
 }

 public void Pickup(){
  m_intakeLeft.set(0.5);
  m_intakeRight.set(-0.5);
 }
 public void StopPickup(){
  m_intakeLeft.set(0);
  m_intakeRight.set(0);
 }
// public void runMotor(double speedbottom, double speedtop){
//     SmartDashboard.putNumber("dataRecorder." + datapoint.ShooterBottom, speedbottom);
//     SmartDashboard.putNumber("dataRecorder." + datapoint.ShooterTop, speedtop);

//     setSpeedBottom = -speedbottom;
//     setSpeedTop = speedtop;

//     if (setSpeedTop == 0 ){
//       m_IntakeLeftPID.setReference(0, CANSparkMax.ControlType.kVoltage);
//       //m_intakeLeft.set(0);
//       //m_IntakeLeftPID.setReference(0, CANSparkMax.ControlType.kPosition);
//       //m_intakeRight.set(0);
//       m_IntakeRightPID.setReference(0, CANSparkMax.ControlType.kVoltage);
//     }
//     else
//     {
//       //m_IntakeLeftPID.
//       //m_intakeLeft.set(setSpeedBottom);
//       //m_intakeRight.set(setSpeedTop); //TalonFXControlMode.Velocity, setSpeedTop);      
//       m_IntakeLeftPID.setReference(setSpeedBottom, CANSparkMax.ControlType.kVelocity);
//       m_IntakeRightPID.setReference(setSpeedBottom, CANSparkMax.ControlType.kVelocity);
//     }
//   }

//   public void setForceReady(boolean enableOverride){
//     manualForceReady = enableOverride;
//   }
//   public void runKicker(double Speed){
//     //m_flipArm.set(ControlMode.PercentOutput, Speed);
//     SmartDashboard.putNumber("dataRecorder." + datapoint.KickerSpeed, Speed);
//   }
//   public boolean isReady(){
//     if (manualForceReady) {
//       return true;
//     }

//     // boolean topReady = (setSpeedTop!=0 && Math.abs(m_intakeRight.getSelectedSensorVelocity() - setSpeedTop) <= 100);
//     // boolean bottomReady = (setSpeedBottom!=0 && Math.abs(m_intakeLeft.getSelectedSensorVelocity() - setSpeedBottom) <= 100);
    
//     // if (topReady && bottomReady) {readyCounter += 1; }
//     // //else {readyCounter = 0; }

//     // if (cacheTopReady != topReady) {cacheTopReady=topReady; SmartDashboard.putBoolean("i T Ready", topReady); }
//     // if (cacheBottomReady != bottomReady) {cacheBottomReady=bottomReady; SmartDashboard.putBoolean("i B Ready", bottomReady); }
    
//     return (readyCounter > 3); //require 10 consecutive readies before reporting we are ready
//   }

//   public void clearReadyFlags(){
//     // cacheTopReady=false;
//     // cacheBottomReady=false;
//     readyCounter = 0;
//     //SmartDashboard.putString("ResetShooter", "Reset");
//   }
}