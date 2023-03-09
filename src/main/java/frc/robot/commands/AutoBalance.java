package frc.robot.commands;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.SwerveModuleMK3;

public class AutoBalance extends CommandBase {

  private final SwerveDrivetrain m_drivetrain;

  // private final Joystick rightController;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  public AutoBalance(SwerveDrivetrain drivetrain) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    m_drivetrain = drivetrain;

    addRequirements(drivetrain);

    // this.rightController = controller2;
  }

  @Override
  public void execute() {

    double BalanceCorrection = 0;
    double checkGyro = 0;

    checkGyro = m_drivetrain.getRoll();
    if (Math.abs(checkGyro) > 3 ) {
      BalanceCorrection = checkGyro * 0.15;
    }
    else {
       checkGyro = m_drivetrain.getPitch();
       if (Math.abs(checkGyro) > 3 ) {
          BalanceCorrection = checkGyro * 0.15;
       }
      }

      if (BalanceCorrection < -0.15) {BalanceCorrection = -0.1; }
      if (BalanceCorrection > 0.15) {BalanceCorrection = 0.1; }

     SmartDashboard.putNumber("BalanceCorrection", BalanceCorrection);
    
     if (Math.abs(BalanceCorrection) < 0.02){
       m_drivetrain.xFormat();
     }
     else {
      final var xSpeed =
      xspeedLimiter.calculate(BalanceCorrection)
        * DriveConstants.kMaxSpeedMetersPerSecond;
    
      m_drivetrain.drive(xSpeed, 0.0, 0.0, true);//, calibrate);
     }

  }
}
