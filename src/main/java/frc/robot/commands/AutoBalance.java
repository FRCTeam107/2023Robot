package frc.robot.commands;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.SwerveModuleMK3;

public class AutoBalance extends CommandBase {

  private final SwerveDrivetrain m_drivetrain;

  // private final Joystick rightController;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  int balanceWaittimeout = 0;
  int switchBackCounter = 0;

  public AutoBalance(SwerveDrivetrain drivetrain) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    m_drivetrain = drivetrain;

    addRequirements(drivetrain);

    // this.rightController = controller2;
  }
 
  @Override
  public void initialize() {
    balanceWaittimeout = 0;
    switchBackCounter = 0;
  }

  @Override
  public void execute() {

    double BalanceCorrection = 0;
    double checkGyro = 0;
    double multiplier;

    if (Math.abs(m_drivetrain.getAngle() % 360) <= 90){
      multiplier = -0.006;
    }
    else {
      multiplier = 0.006;
    }


    checkGyro = m_drivetrain.getRoll();
    if (Math.abs(checkGyro) > 6 ) {
      BalanceCorrection = checkGyro * multiplier;
    }
    else {
       checkGyro = m_drivetrain.getPitch();
       if (Math.abs(checkGyro) > 6 ) {
          BalanceCorrection = checkGyro * multiplier;
       }
      }

    if (BalanceCorrection < -0.07) {BalanceCorrection = -0.07; }
    if (BalanceCorrection > 0.07) {BalanceCorrection = 0.07; }

    SmartDashboard.putNumber("Gyro Roll", m_drivetrain.getRoll());
    SmartDashboard.putNumber("Gyro Pitch", m_drivetrain.getPitch());
    SmartDashboard.putNumber("BalanceCorrection", BalanceCorrection);
    //  SmartDashboard.putNumber("BalanceTimeout", balanceWaittimeout);

     if (Math.abs(BalanceCorrection) < 0.02){
       m_drivetrain.xFormat();
       balanceWaittimeout = 50;
       switchBackCounter = 0;
     }
     else {
      if (balanceWaittimeout <= 0){
        double balanceSpeed = xspeedLimiter.calculate(BalanceCorrection)
              * DriveConstants.kMaxSpeedMetersPerSecond;
        switchBackCounter = (switchBackCounter + 1) % 150;
        if (switchBackCounter < 75){
          m_drivetrain.drive(balanceSpeed, balanceSpeed/0.9, 0.0, true);//, calibrate);
        }
        else {
          m_drivetrain.drive(balanceSpeed, balanceSpeed/-0.9, 0.0, true);//, calibrate);
        }
        
      }
      else {
        balanceWaittimeout --;
      }

     }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
