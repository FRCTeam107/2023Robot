/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonPause extends CommandBase {
  /**
   * Creates a new Shoot.
   */
  private double m_pauseTime;
  private double startTime;
  
  public AutonPause(double pauseSeconds) {
    // m_pauseTime = _pauseTime;
    m_pauseTime = pauseSeconds;

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements();
  }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Limelight.EnableVisionProcessing();
    if (m_pauseTime < 0) {
      // m_pauseTime = SmartDashboard.getNumber("Auton Wait", 1);
      // SmartDashboard.putNumber("Auton Wait", m_pauseTime);
    }
    startTime = Timer.getFPGATimestamp();
    System.out.println("------>>>>>> Begin Auton pause at " + String.valueOf(startTime));
    //if (m_pauseTime > 0) { Timer.delay(m_pauseTime);}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Timer.delay(m_pauseTime);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("------>>>>>> End Auton pause at " + String.valueOf(Timer.getFPGATimestamp()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Timer.getFPGATimestamp() - startTime) > m_pauseTime);
  }
}
