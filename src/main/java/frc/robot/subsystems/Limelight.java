/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.LinkedList;
// import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  private final double kRangeTx = 1.5;  // acceptable amount X can be off to hit the shot
  private final double kRangetY = 0.4;  // acceptable amount Y can be off to hit the shot
 private final double kRangetA_Low = 0.21;
 private final double kRangetA_High = 0.31;

  private final NetworkTable table;
  //private final NetworkTableEntry tV, tX, tY, tA,
  private final NetworkTableEntry ledMode, camMode, pipeline;

  private boolean m_visionProcessingEnabled, m_overrideEnabled;
  //    m_LEDisON = false;
  private double m_tV=0;
  private double m_tX=0; 
  private double m_tY=0;
  private double m_tA=0;
  private double lastTX, lastTY, lastTA;
  private double cacheTX, cacheTY, cacheTA, cacheTV;

  // private double x_offset;
  // private List<Double> txValues;
  //private double m_AvgTx, m_AvgTy, m_AvgTa;

  public Limelight(){
    super();

    // x_offset  = SmartDashboard.getNumber("X-offset", 0.0);
    // SmartDashboard.putNumber("X-offset", x_offset);

    //SmartDashboard.putNumber("Limelight enabled", 0);
    m_overrideEnabled = false;
    m_visionProcessingEnabled = false;
    table = NetworkTableInstance.getDefault().getTable("limelight");
    // tV = table.getEntry("tv");
    // tX = table.getEntry("tx");
    // tY = table.getEntry("ty");
    // tA = table.getEntry("ta");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
    pipeline = table.getEntry("pipeline");

    this.DisableVisionProcessing();

    // set avg to invalid value to force reset
    lastTA = 999;
    lastTX = 999;
    lastTY = 999;
    cacheTA = 0;
    cacheTX = 0;
    cacheTY = 0;
    cacheTV = 0;
  }

  @Override
  public void periodic() {
    // x_offset  = SmartDashboard.getNumber("X-offset", 0.0);
    if (m_visionProcessingEnabled)
    {
      m_tV = this.tV();
      m_tX = this.TX();
      m_tY = this.TY();
      m_tA = this.TA();
    }
    else
    {
      m_tV=0;
      m_tX=0;
      m_tY=0;
      m_tA=0;
    }
    // double tV = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(999);
    // double tY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(999);
    // double tX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(999);
    // SmartDashboard.putNumber("Limelight tX", m_tX);
    
    if (cacheTV != m_tV) {
      cacheTV = m_tV;
      SmartDashboard.putNumber("Limelight tA", m_tA); 
      SmartDashboard.putBoolean("Limelight Target", Havetarget());
    }
    //SmartDashboard.putNumber("Limelight tV", m_tY);
    if (cacheTX != m_tX) {cacheTX=m_tX;  SmartDashboard.putNumber("Limelight tX", m_tX); }
    if (cacheTY != m_tY) {cacheTY=m_tY; SmartDashboard.putNumber("Limelight tY", m_tY); }
    if (cacheTA != m_tA) {cacheTA=m_tA; SmartDashboard.putNumber("Limelight tA", m_tA); }
  }

  public double tV() {
    return  NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  }
  
  public double rawTX () {
    return table.getEntry("tx").getDouble(0);
  }
  public double TX(){
    return  rawTX();
    //  double thisTX = rawTX(); // - x_offset;
    //  double retval;
    //  //if (lastTX==999){ 
    //  if ( Math.abs(lastTX) > 5){  /// was 999 
    //    retval = thisTX;
    //  }
    //  else {
    //    retval = thisTX; //(thisTX + lastTX) / 2.0;
    //  }
    //  lastTX = thisTX;
    //  return retval;
  }

  public double rawTY(){
    return  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double TY(){
    return rawTY();
    // double thisTY = rawTY();
    // double retval;
    // if (lastTX==999){ 
    //   retval = thisTY;
    // }
    // else {
    //   retval = thisTY + lastTY / 2.0;
    // }
    // lastTX = thisTY;
    // return retval;
  }

  public double rawTA(){
    return  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }
  public double TA(){
    //return rawTA();
    double thisTA = rawTA();
    double retval;
    if (lastTA==999){ 
      retval = thisTA;
    }
    else {
      retval = (thisTA + lastTA) / 2.0;
    }
    lastTA = thisTA;
    return retval;
  }

  public boolean Havetarget(){
    // return true if we have a target (and vision processing enabled)
    if (!m_visionProcessingEnabled){ return false; }
    return(this.m_tV==1);
  }

  public void TurnOnLED(){
    ledMode.setNumber(3);
    // m_LEDisON = true;
  }

  public void TurnOffLED(){
    ledMode.setNumber(1);
  // m_LEDisON = false;
  }

  public void EnableVisionProcessing(){
    TurnOnLED();
    //SmartDashboard.putNumber("Limelight enabled", 1);
    camMode.setNumber(0);
    //pipeline.setNumber(2);  //2 for shooting?
    m_visionProcessingEnabled = true;
  }

  public void DisableVisionProcessing(){
    m_visionProcessingEnabled=false;
    camMode.setNumber(1);
    //pipeline.setNumber(2);
    TurnOffLED();
    // set avg to invalid value to force reset
    lastTA = 999;
    lastTX = 999;
    lastTY = 999;
  }

  // public void ToggleVisionProcessing(){
  //   if (m_visionProcessingEnabled){
  //     DisableVisionProcessing();
  //   }
  //   else {
  //     EnableVisionProcessing();
  //   }
  // }

  // public void setForceReady(boolean enableOverride){
  //   m_overrideEnabled = enableOverride;
  // }

  public boolean XinRange(){
    return (Havetarget() && Math.abs(m_tX) <= kRangeTx);
  }
  public boolean YinRange(){
    return (Havetarget() && Math.abs(m_tY) <= kRangetY);
  }
  public boolean AinRange(){
    return (Havetarget() && Math.abs(m_tA) >= kRangetA_Low  && Math.abs(m_tA) <= kRangetA_High );
  }
  public boolean isReady(){
    if (m_overrideEnabled) {return true;}

    return (XinRange()) && AinRange();// && YinRange());
  }
}