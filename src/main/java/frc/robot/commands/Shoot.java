/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private final Shooter m_shoot;
  private final BooleanSupplier m_forceShot;
  private final BooleanSupplier m_turboShot;
  private final Limelight m_Limelight;

  private double iTop, iBottom;
 
  public Shoot(Shooter _shooter, Limelight _limeLight, BooleanSupplier _forceShot, BooleanSupplier _turboShot) {
    m_shoot = _shooter;
    m_forceShot = _forceShot;
    m_turboShot = _turboShot;
    m_Limelight = _limeLight;

    iTop = SmartDashboard.getNumber("i Top", 7500);
    iBottom = SmartDashboard.getNumber("i Bottom", 7500);
    SmartDashboard.putNumber("i Top", iTop);
    SmartDashboard.putNumber("i Bottom", iBottom);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shoot);
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedTop = 8000;
    double speedBottom = 10000;
    double liftPosition = 0;
    
    speedTop = SmartDashboard.getNumber("i Top", iTop);
    speedBottom = SmartDashboard.getNumber("i Bottom", iBottom);
    liftPosition = SmartDashboard.getNumber("i Lift", liftPosition);


    // if (m_Limelight.Havetarget()) {
    // //   // double m_tX = m_Limelight.tX();   // how far off, the 'X' value
    // //   // double m_tY = m_Limelight.tY();   //how far off, the 'Y' value
    //   double tA = m_Limelight.TA();   //distance from target estimation
    //   double adjustSpeed = 0;
    //   if (tA < 0.234){
    //     adjustSpeed = (0.234 - tA) * 3000;
    //   }
    //   else if (tA > 0.234){
    //     adjustSpeed = (0.234 - tA) * 7800;
    //   }
    //   if (Math.abs(adjustSpeed) > 800){ adjustSpeed = Math.signum(adjustSpeed) * 800; }
    //   speedBottom += adjustSpeed;
    //   speedTop += adjustSpeed;
    // }

    if (m_turboShot.getAsBoolean()){
      m_shoot.runMotor(11000 ,11000);
    }
    else {
      m_shoot.runMotor(speedBottom ,speedTop);
    }
   // SmartDashboard.putNumber("ShooterSppeed", speedTop);
    // SmartDashboard.putNumber("mShoot Bot", speedBottom);
   // m_turret.setLifterPosition(liftPosition);


    if (m_forceShot.getAsBoolean()){ //} || (m_shoot.isReady()) ) { //&& m_turret.isLiftReady()) ) { //&& m_Limelight.isReady()) ){
     m_shoot.runKicker(0.3);
     // m_Hopper.runMotor(0.41);

    //   double tt = Timer.getFPGATimestamp();
    //    if ((int)(tt * 10) % 3 == 0) {
    //    //  m_Hopper.runMotor(0.35);
    //    }
    //    else {
    //     // m_Hopper.runMotor(0);
    //    }
    }
     else
    { 
      m_shoot.runKicker(0);
      // m_Indexer.runMotor(-0.3);
      // m_Hopper.runMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_turret.isShooting = false;
    m_shoot.runMotor(0, 0);
    m_shoot.clearReadyFlags();
    m_shoot.runKicker(0);
    //m_Indexer.runMotor(0);
    //m_Hopper.runMotor(0);
    //m_Limelight.DisableVisionProcessing();

    // SmartDashboard.putNumber("mShoot Top", 0);
    // SmartDashboard.putNumber("mShoot Bot", 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
