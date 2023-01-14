/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.Motors;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DataRecorder.datapoint;

public class Shooter extends SubsystemBase {
  private final WPI_TalonFX m_shootbottom, m_shoottop;
  private final WPI_TalonSRX m_kicker;

  private double setSpeedTop, setSpeedBottom;

  // private double kP, kI, kD, kFF, kMaxOutput, kMinOutput, maxRPM;
  // private int kIz;
  private boolean manualForceReady;
  private int readyCounter;  // number of times shooters report ready in a row

  private boolean cacheTopReady, cacheBottomReady;
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    super();

    manualForceReady = false;
    cacheBottomReady = false;
    cacheTopReady = false;

    readyCounter = 0;
    m_shootbottom = new WPI_TalonFX(Motors.SHOOTER_BOTTOM);
    m_shoottop = new WPI_TalonFX(Motors.SHOOTER_TOP);
    m_kicker = new WPI_TalonSRX(Motors.KICKER);

    m_shootbottom.configFactoryDefault();
    m_shoottop.configFactoryDefault();
    m_kicker.configFactoryDefault();


    m_shoottop.setStatusFramePeriod(StatusFrame.Status_1_General, 100);
    m_shootbottom.setStatusFramePeriod(StatusFrame.Status_1_General, 100);

    m_shoottop.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);
    m_shootbottom.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

    m_kicker.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    m_kicker.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);

    // m_shootbottom.configClosedloopRamp(0.3);
    // m_shoottop.configClosedloopRamp(0.3);
   
    setSpeedTop = 0;
    setSpeedBottom = 0;

    // setup PID closed-loop values
    // kP = 0.25; 
    // kI = 0.0005;
    // kD = 0.0001; 
    // kIz = 8000; 
    // kFF = 0;//.000015; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;
    m_shootbottom.config_kP(0, ShooterConstants.kP);
    m_shootbottom.config_kI(0, ShooterConstants.kI);
    m_shootbottom.config_kD(0, ShooterConstants.kD);
    m_shootbottom.config_IntegralZone(0, ShooterConstants.kIz);
    m_shootbottom.config_kF(0, ShooterConstants.kFF);
    // m_shootbottom.configClosedLoopPeakOutput(slotIdx, percentOut)

    m_shoottop.config_kP(0, ShooterConstants.kP);
    m_shoottop.config_kI(0, ShooterConstants.kI);
    m_shoottop.config_kD(0, ShooterConstants.kD);
    m_shoottop.config_IntegralZone(0, ShooterConstants.kIz);
    m_shoottop.config_kF(0, ShooterConstants.kFF);
    // m_shoottop.configClosedLoopPeakOutput(slotIdx, percentOut)

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 }

 
public void runMotor(double speedbottom, double speedtop){
    SmartDashboard.putNumber("dataRecorder." + datapoint.ShooterBottom, speedbottom);
    SmartDashboard.putNumber("dataRecorder." + datapoint.ShooterTop, speedtop);

    setSpeedBottom = -speedbottom;
    setSpeedTop = speedtop;

    if (setSpeedTop == 0 ){
      m_shootbottom.set(TalonFXControlMode.PercentOutput, 0);
      m_shoottop.set(TalonFXControlMode.PercentOutput,0);
    }
    else
    {
      m_shootbottom.set(TalonFXControlMode.Velocity, setSpeedBottom);
      m_shoottop.set(TalonFXControlMode.Velocity, setSpeedTop);      
    }
  }

  public void setForceReady(boolean enableOverride){
    manualForceReady = enableOverride;
  }
  public void runKicker(double Speed){
    m_kicker.set(ControlMode.PercentOutput, Speed);
    SmartDashboard.putNumber("dataRecorder." + datapoint.KickerSpeed, Speed);
  }
  public boolean isReady(){
    if (manualForceReady) {
      return true;
    }

    boolean topReady = (setSpeedTop!=0 && Math.abs(m_shoottop.getSelectedSensorVelocity() - setSpeedTop) <= 100);
    boolean bottomReady = (setSpeedBottom!=0 && Math.abs(m_shootbottom.getSelectedSensorVelocity() - setSpeedBottom) <= 100);
    
    if (topReady && bottomReady) {readyCounter += 1; }
    //else {readyCounter = 0; }

    if (cacheTopReady != topReady) {cacheTopReady=topReady; SmartDashboard.putBoolean("i T Ready", topReady); }
    if (cacheBottomReady != bottomReady) {cacheBottomReady=bottomReady; SmartDashboard.putBoolean("i B Ready", bottomReady); }
    
    return (readyCounter > 3); //require 10 consecutive readies before reporting we are ready
  }

  public void clearReadyFlags(){
    cacheTopReady=false;
    cacheBottomReady=false;
    readyCounter = 0;
    //SmartDashboard.putString("ResetShooter", "Reset");
  }
}