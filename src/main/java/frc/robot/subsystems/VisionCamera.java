/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionCamera extends SubsystemBase {
  /**
   * Creates a new visioncamera.
   */
  private final Servo m_Servo = new Servo(0);

   public VisionCamera() {

  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("cameraPos", m_Servo.getAngle());
    // This method will be called once per scheduler run
    m_Servo.setAngle( SmartDashboard.getNumber("cameraPos", m_Servo.getAngle()));
  }
   public void moveServoToPosition(double position){
     m_Servo.setAngle(position);
   }

   public void raiseCamera(){
    m_Servo.setAngle(135);
   }

   public void lowerCamera(){
     m_Servo.setAngle(0);
    }

   public void middleCamera(){
     m_Servo.setAngle(45);
   }

   public void changeCamera(){
    if(m_Servo.getAngle() <= 31){m_Servo.setAngle(180);}
    else{m_Servo.setAngle(30);}}
   
}

