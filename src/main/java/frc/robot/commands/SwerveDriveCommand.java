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

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain m_drivetrain;
  private final Joystick m_FlightController;
  private final Limelight m_Limelight;
  private final LEDLights m_LEDLights;

  // private final Joystick rightController;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, Joystick controller,  Limelight _limeLight, LEDLights _LEDLights) {
  //public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    m_drivetrain = drivetrain;
    m_Limelight = _limeLight;
    m_FlightController = controller;
    m_LEDLights = _LEDLights;

    addRequirements(drivetrain);

    // this.rightController = controller2;
  }

  @Override
  public void execute() {
    // SmartDashboard.putNumber("X channel", FlightController.getX());
    // SmartDashboard.putNumber("Y channel", FlightController.getY());
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double X = -m_FlightController.getX();
    if (Math.abs(X)<0.09){
      X = 0;
    }
    double Y = m_FlightController.getY();
    if (Math.abs(Y)<0.09){
      Y = 0;
    }

    double Z = m_FlightController.getZ();
    if (Math.abs(Z)<0.07){
      Z = 0;
    }

    // if driver is pressing aim by limelight button, then use limelight if possible
    if (m_FlightController.getRawButton(Constants.FlightController.AIM_BY_LIMELIGHT)){
      if (m_Limelight.Havetarget() ){
        Z =  -m_Limelight.TX() / 27 * 0.9;// 1.3 * 2;
        if (Z<-1){Z=-1;}
        else if(Z>1){Z=1;}
      }
    }

    // if limelight has target and x-axis is in range to shoot, turn lights green for 5 20ms cycles
    if (m_Limelight.isReady()){
      m_LEDLights.lightsGreen(5);
    }

    final var xSpeed =
      xspeedLimiter.calculate(Y)
        * DriveConstants.kMaxSpeedMetersPerSecond;
    
   // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
      yspeedLimiter.calculate(X)
        * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
       rotLimiter.calculate(Z)
         * DriveConstants.kMaxAngularSpeed;
    // final var rot =
    //     -rotLimiter.calculate(0) //leftController.getZ())
    //       * SwerveDrivetrain.kMaxAngularSpeed;
    //boolean calibrate = false; //controller.getLeftBumper();
//SmartDashboard.putBoolean("calibrate", calibrate);

    m_drivetrain.drive(xSpeed, ySpeed, rot, true);//, calibrate);
   
  }

}
