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

public class DriveAuto extends CommandBase {

  private final SwerveDrivetrain m_drivetrain;
  private final double X_Drive, Y_Strafe, m_FaceHeading, m_Distance, m_StartingPosition;

  public DriveAuto(SwerveDrivetrain drivetrain, double speed, double driveAngle, double faceAngle, double distance) {

    m_drivetrain = drivetrain;
    // requested drive speed is hypotenuse of right angle, calculate X and Y as if it is joystick
    X_Drive = Math.cos(driveAngle) * speed;
    Y_Strafe = Math.sin(driveAngle) * speed;

    m_Distance = distance;
    m_FaceHeading = faceAngle;

    // get current distance recorded on wheels to determine if we drove far enough
    m_StartingPosition = 0;
    //m_StartingPosition = m_drivetrain.get

    addRequirements(drivetrain);

    // this.rightController = controller2;
  }

  @Override
  public void execute() {
       //m_drivetrain.xFormat();
       // caclulate rotation needed to achieve desired facingAngle
       double Z_Rotate = 0;
       
       double currentGyrAngle = m_drivetrain.getAngle();
       
       // negative Z turns clockwise (increasing gyro angle)
       double adjustZ = (m_FaceHeading - currentGyrAngle) * -0.05;
       if (adjustZ < -0.2) { adjustZ = -0.2;}
       if (adjustZ > 0.2) { adjustZ = 0.2;}
       Z_Rotate += adjustZ;  

       m_drivetrain.drive(X_Drive, Y_Strafe, Z_Rotate, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: get current distance on drivetrain and see if we went far enough to be done
      return false;
  }
}
