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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.subsystems.DataRecorder.datapoint;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX m_IntakeMotor;
  private final WPI_TalonSRX m_IntakeArm;

  private boolean intakeExtended = false;
  private double m_Intake_ArmSpeed;
  private double m_IntakeSpeed;

  //private DataRecorder m_dataRecorder;
  
  public static final class IntakeArmConstants {
    //run arm motor to extended position, find right position
    public static final double armStartingPos = 0;
   //public static final double armRetractPos = 1000;
    public static final double armStopRetract = -2500;
    public static final double armSlowRetract = -25000;
    public static final double armExtendedPos = -67000;
    
    // intake arm PID values
    public static final double kP =2; // 0.4; //0.04; 
    public static final double kI = 0; //0.0002;
    public static final double kD = 0.0; 
    public static final double kIz = 0; //8000; 
    public static final double kFF = 8; //0.1;// 0;//.000015; 
    public static final double kMaxOutput = 0.2; // 0.3; 
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700;  
}

  public static final class IntakeMotorConstants {

    public static final double kP = 0.04; 
    public static final double kI = 0.00;
    public static final double kD = 0; 
    public static final double kIz = 8000; 
    public static final double kFF = 0;//.000015; 
    // public static final double kMaxOutput = 1; 
    // public static final double kMinOutput = -1;
    // public static final double maxRPM = 5700; 

  }
  /**
   * Creates a new Intake.
   */
  public Intake() {
    m_IntakeMotor = new WPI_TalonSRX(Motors.BALL_INTAKE);
    m_IntakeMotor.configFactoryDefault();
    m_IntakeMotor.setInverted(false);
    m_IntakeMotor.configClosedloopRamp(0.5);

    
    m_IntakeMotor.config_kP(0, IntakeMotorConstants.kP);
    m_IntakeMotor.config_kI(0, IntakeMotorConstants.kI);
    m_IntakeMotor.config_kD(0, IntakeMotorConstants.kD);
    m_IntakeMotor.config_IntegralZone(0, IntakeMotorConstants.kIz);
    m_IntakeMotor.config_kF(0, IntakeMotorConstants.kFF);

    m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    m_IntakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);

    double junk = SmartDashboard.getNumber("intakeSpeed", 35000);
    SmartDashboard.putNumber("intakeSpeed", junk);

    m_IntakeSpeed = 0;
   
    m_IntakeArm = new WPI_TalonSRX(Motors.INTAKE_ARM);
    m_IntakeArm.configFactoryDefault();
    m_IntakeArm.setInverted(false);
    m_IntakeArm.setNeutralMode(NeutralMode.Brake);
    m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armStartingPos);
    // PID values for INTAKE_ARM
    m_IntakeArm.config_kP(0, IntakeArmConstants.kP);
    m_IntakeArm.config_kI(0, IntakeArmConstants.kI);
    m_IntakeArm.config_kD(0, IntakeArmConstants.kD);
    m_IntakeArm.config_IntegralZone(0, IntakeArmConstants.kIz);
    m_IntakeArm.config_kF(0, IntakeArmConstants.kFF);
    m_IntakeArm.configClosedLoopPeakOutput(0, IntakeArmConstants.kMaxOutput);

    // m_IntakeArm.configClearPositionOnLimitR(clearPositionOnLimitR, timeoutMs)
    // m_IntakeArm.configClearPositionOnLimitF(clearPositionOnLimitF, timeoutMs)

    m_IntakeArm.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    m_IntakeArm.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

   // m_dataRecorder = null;
    intakeExtended = false;
    m_Intake_ArmSpeed = 0;
  }

  // public void setDataRecorder(DataRecorder _dataRecorder){
  //   m_dataRecorder = _dataRecorder;
  // }

   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeArmAt", m_IntakeArm.getSelectedSensorPosition());
    SmartDashboard.putBoolean("IntakeFwdLimit", m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("IntakeRevLimit", m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed());

    // runMotor(m_CurrentSpeed);
        //m_IntakeMotor.set(ControlMode.PercentOutput, m_CurrentSpeed);
    m_IntakeMotor.set(ControlMode.Velocity, m_IntakeSpeed);

    //NetworkTableInstance.getDefault().getTable("dataRecorder").
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
    if (m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed()){
      m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armStartingPos);
    }
    
    if (intakeExtended){
      if (m_IntakeArm.getSensorCollection().isRevLimitSwitchClosed() 
          || m_IntakeArm.getSelectedSensorPosition() <= IntakeArmConstants.armExtendedPos){
        m_Intake_ArmSpeed = 0;
        stopArm();
      }
      else if (m_Intake_ArmSpeed < 0 && m_IntakeArm.getSelectedSensorPosition() <  -10000) {
        m_Intake_ArmSpeed = -0.55;  // speed up on lower end of extending arm
      }
    }
    else {
      if (m_IntakeArm.getSensorCollection().isFwdLimitSwitchClosed() 
        || m_IntakeArm.getSelectedSensorPosition() >= IntakeArmConstants.armStopRetract ){
      //   m_IntakeArm.setSelectedSensorPosition(IntakeArmConstants.armExtendedPos);
        stopArm();
        m_Intake_ArmSpeed = 0;
      }
      else if (m_Intake_ArmSpeed > 0 && m_IntakeArm.getSelectedSensorPosition() >= IntakeArmConstants.armSlowRetract) {
        m_Intake_ArmSpeed = 0.15;  // slow down as we approach closed position
      }
    }
    m_IntakeArm.set(ControlMode.PercentOutput, m_Intake_ArmSpeed);  
  
  }
  // public void runMotor(double speed){
  //   //m_IntakeMotor.set(ControlMode.PercentOutput, speed);
  //   m_IntakeMotor.set(ControlMode.Velocity, speed);
  // }
public void extendArm(){
  // m_IntakeArm.set(ControlMode.PercentOutput, 1);
  SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 1.0);
  //m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armExtendedPos);
  intakeExtended = true;
  m_IntakeMotor.set(ControlMode.PercentOutput, -0.3);
  m_Intake_ArmSpeed = -0.20;// -IntakeArmConstants.kMaxOutput;
  // if (m_dataRecorder != null) {
  //   m_dataRecorder.recordValue(datapoint.IntakeIsExtended, (double)1.00);
  // }
}

public void retractArm(){
  //m_IntakeArm.set(ControlMode.PercentOutput, -1);
  SmartDashboard.putNumber("dataRecorder." + datapoint.IntakeIsExtended, 0.0);

  //m_IntakeArm.set(ControlMode.Position, IntakeArmConstants.armRetractPos);
  intakeExtended = false;
  m_Intake_ArmSpeed = 0.55;// IntakeArmConstants.kMaxOutput;
  
  // if (m_dataRecorder != null) {
  //   m_dataRecorder.recordValue(datapoint.IntakeIsExtended, (double)0.00);
  // }

}

public void stopArm() {
  m_IntakeArm.set(ControlMode.PercentOutput, 0);
  m_Intake_ArmSpeed = 0;
}

public void runIntake(double speed){
    m_IntakeSpeed = speed;
  }

  public void HeimlichManeuver() {
    runIntake( -1 * SmartDashboard.getNumber("intakeSpeed", 25000));
  }
  public void StopIntake() {
    runIntake(0);
  }
  public void StartIntake() {
    runIntake(SmartDashboard.getNumber("intakeSpeed", 25000));
  }

  public void allowAdditionalMovement(){
    m_IntakeArm.setSelectedSensorPosition( IntakeArmConstants.armExtendedPos / 2);
  }

}