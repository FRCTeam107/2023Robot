// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.channels.WritableByteChannel;
import java.util.List;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControllerJoystick;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Constants.FlightController;
import frc.robot.commands.SkyHook_MoveWrist;
import frc.robot.commands.SkyHook_RunIntake;
import frc.robot.commands.SkyHook_Scoring;
import frc.robot.commands.SkyHook_ScoringPlus;
import frc.robot.commands.SkyHook_MoveArm;
import frc.robot.commands.SkyHook_MoveElevator;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutonPause;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.LED_ColorSet;
import frc.robot.commands.ReplayFile;
//import frc.robot.commands.ReplayFile;
import frc.robot.commands.SetRobotOrientationOnField;
//import frc.robot.commands.Shoot;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DataRecorder;
//import frc.robot.subsystems.Intake;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.XFactor;
//import frc.robot.commands.TransferToNextBar;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.VisionCamera;
import frc.robot.subsystems.SkyHook;
//import frc.robot.subsystems.VisionCamera;
import frc.robot.subsystems.SkyHook.ArmPositions;
import frc.robot.subsystems.SkyHook.ExtensionPositions;
//import frc.robot.commands.ClimberResetToHome;
//import frc.robot.commands.DismountFirstBar;
//import frc.robot.commands.ReachForTheBar;
//import frc.robot.commands.PullUpOntoTalonHooks;
//import frc.robot.commands.RunClimberManually;
import frc.robot.subsystems.SkyHook.WristPositions;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private final Joystick m_leftJoystick, m_rightJoystick, m_controllerJoystick;
  private final Joystick m_flightcontroller, m_controllerJoystick;
  private final SwerveDrivetrain m_Drivetrain;
  private final LEDLights m_LEDLights;
  //private final VisionCamera m_Camera;
  private final Limelight m_limelight;
  private final SkyHook m_skyHook;
  
  public DataRecorder m_DataRecorder = new DataRecorder();

  // private final Compressor m_compressor;

      // A chooser for autonomous commands
  private final SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    
    m_flightcontroller = new Joystick(Constants.UsbPorts.FlightController);
    m_flightcontroller.setXChannel(FlightController.DRIVE_X_AXIS);
    m_flightcontroller.setYChannel(FlightController.DRIVE_Y_AXIS);
    m_flightcontroller.setZChannel(FlightController.DRIVE_Z_AXIS);
    // m_limit = new PWM(0);
    m_skyHook = new SkyHook();

    m_controllerJoystick = new Joystick(Constants.UsbPorts.CONTROLLER_STICK);
    m_LEDLights = new LEDLights();
    m_Drivetrain  = new SwerveDrivetrain(0);  // begin assuming no field offset angle of robot (facing straight "north")

    m_limelight = new Limelight();

    m_DataRecorder = new DataRecorder();
    
    m_Drivetrain.setDefaultCommand(new SwerveDriveCommand(m_Drivetrain, m_flightcontroller, m_limelight, m_LEDLights));
    //m_Drivetrain.setDefaultCommand(new AutoBalance(m_Drivetrain));
    
    configureButtonBindings();

    // Add commands to the autonomous command chooser


    // Command CenterCubeBalance = new SequentialCommandGroup(
    //     new SetRobotOrientationOnField(m_Drivetrain, 0.0),
    //     new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
    //     new SetRobotOrientationOnField(m_Drivetrain, 0.0),
    //     new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "centerbalance.csv"),
    //     new AutoBalance(m_Drivetrain)
    //     );

    Command LowConeBal = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, -180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "lowconebal.csv"),
          new AutoBalance(m_Drivetrain)
          );
    Command CubeWithOverAndBackFront = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, -180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHighFront.csv"),
          new SetRobotOrientationOnField(m_Drivetrain, -180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "overback180.csv"),
          new AutoBalance(m_Drivetrain)
          );
      Command CenterCubeFrontBalanceFast = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, 180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHighFront.csv"),
          new SetRobotOrientationOnField(m_Drivetrain, 180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "centerbalancefast180.csv"),
          new AutoBalance(m_Drivetrain)
          );
      Command CenterFrontBalanceOnly = new SequentialCommandGroup(
            new SetRobotOrientationOnField(m_Drivetrain, 180.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "centerbalancefast180.csv"),
            new AutoBalance(m_Drivetrain)
            );
  
    // Command CenterConeBalance = new SequentialCommandGroup(
    //       new SetRobotOrientationOnField(m_Drivetrain, 0.0),
    //       new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreConeHigh.csv"),
    //       new SetRobotOrientationOnField(m_Drivetrain, 0.0),
    //       new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "centerbalance.csv"),
    //       new AutoBalance(m_Drivetrain)
    //       );

     Command CenterConeFrontBalanceFast = new SequentialCommandGroup(
            new SetRobotOrientationOnField(m_Drivetrain, 180.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreConeHighFront.csv"),
            new SetRobotOrientationOnField(m_Drivetrain, 180.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "centerbalancefast180.csv"),
            new AutoBalance(m_Drivetrain)
            );
    Command CubeFrontAndRun = new SequentialCommandGroup(
        new SetRobotOrientationOnField(m_Drivetrain, 180.0),
        new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHighFront.csv"),
        new SetRobotOrientationOnField(m_Drivetrain, 180.0),
        new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "SimpleStraightRun180.csv")
       );

    Command TwoCubeBumpBLUE = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, 0.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
          new SetRobotOrientationOnField(m_Drivetrain, 0.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "runforcubeBump_BLUE.csv")
           );

      Command TwoCubeBumpRED = new SequentialCommandGroup(
            new SetRobotOrientationOnField(m_Drivetrain, 0.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
            new SetRobotOrientationOnField(m_Drivetrain, 0.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "runforcubeBump_RED.csv")
             );

    Command TwoCubeMidX = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, 0.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
          new SetRobotOrientationOnField(m_Drivetrain, 0.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "runforcubemid.csv")
          );
    
    Command TwoCubeMidBlue = new SequentialCommandGroup(
            new SetRobotOrientationOnField(m_Drivetrain, 0.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
            new SetRobotOrientationOnField(m_Drivetrain, 0.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "runforcubemid_BLUE.csv")
            );

     Command TwoCubeMidREd = new SequentialCommandGroup(
            new SetRobotOrientationOnField(m_Drivetrain, 0.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
            new SetRobotOrientationOnField(m_Drivetrain, 0.0),
            new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "runforcubemid_RED.csv")
            );
      
    Command OneConeFrontRun = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, 180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreConeHighFront.csv"),
          new SetRobotOrientationOnField(m_Drivetrain, 180.0),
          new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "SimpleStraightRun180.csv")
          );

      Command CubeFrontAndStayFront = new SequentialCommandGroup(
        new SetRobotOrientationOnField(m_Drivetrain, 180.0),
        new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHighFront.csv"),
        new SetRobotOrientationOnField(m_Drivetrain, 180.0) );
        
      Command ConeFrontAndStay = new SequentialCommandGroup(
        new SetRobotOrientationOnField(m_Drivetrain, 180.0),
        new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreConeHighFront.csv"),
        new SetRobotOrientationOnField(m_Drivetrain, 180.0)
        );

      Command testAutoDrive = new SequentialCommandGroup(
        new SetRobotOrientationOnField(m_Drivetrain, 0.0),
        new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "TestRun.csv")
        //        new DriveAuto(m_Drivetrain, 0.2, 90.0, 0.0, 107),
        //new AutonPause(1),
        //new DriveAuto(m_Drivetrain, -0.2, 20.0, 0.0, 50)
        );
    // Command BlueTwoCubeRun = new SequentialCommandGroup(
    //     new SetRobotOrientationOnField(m_Drivetrain, 0.0),
    //     new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "ScoreCubeHigh.csv"),
    //     new SetRobotOrientationOnField(m_Drivetrain, 0.0),
    //     new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "twocube.csv")
    //     );

    Command PlayDead = new SequentialCommandGroup(
          new SetRobotOrientationOnField(m_Drivetrain, 0.0));


    // Add commands to the autonomous command chooser
    m_chooser = new SendableChooser<>();
    //m_chooser.addOption("TwoCubeOverCable", TwoCubeOverCable);
    // m_chooser.addOption("Center Cube+Balance", CenterCubeBalance);
    // m_chooser.addOption("Ctr Cone and Balance", CenterConeBalance);

    m_chooser.addOption("LowCone Front bal", LowConeBal);
    m_chooser.addOption("CubeFront+Over and Back", CubeWithOverAndBackFront);
    m_chooser.addOption("Empty Front Balance Only", CenterFrontBalanceOnly);
    m_chooser.addOption("Cube Front and Balance", CenterCubeFrontBalanceFast);
    m_chooser.addOption("Cone Front and Balance", CenterConeFrontBalanceFast);
    m_chooser.addOption("Cube Front and Run", CubeFrontAndRun);
    m_chooser.addOption("Two Cube - bump BLUE", TwoCubeBumpBLUE);
    m_chooser.addOption("Two Cube - bump RED", TwoCubeBumpRED);
    m_chooser.addOption("Two cube - smooth BLUE", TwoCubeMidBlue);
    m_chooser.addOption("Two cube - smooth RED", TwoCubeMidREd);
    m_chooser.addOption("Two cube - original", TwoCubeMidX);
    m_chooser.addOption("One Cone Front Run", OneConeFrontRun);
    //m_chooser.addOption("2 cube run", BlueTwoCubeRun);
    m_chooser.addOption("Cone Frontand stay", ConeFrontAndStay);
    m_chooser.addOption("Cube Front and stay", CubeFrontAndStayFront);
    m_chooser.addOption("play dead", PlayDead);

    //m_chooser.addOption("test autodrive", testAutoDrive);

    //m_chooser.addOption("Barrel", new Barrel(m_drivetrain));
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  private void configureButtonBindings() {
    // // setup buttons

    JoystickButton btnResetDrivetrainOrientation =  new JoystickButton(m_flightcontroller, FlightController.REORIENT_ROBOT);

    JoystickButton btnAutoBalance = new JoystickButton(m_flightcontroller, FlightController.AUTO_BALANCE);    
    btnAutoBalance.whileTrue(new AutoBalance(m_Drivetrain));

    JoystickButton btnXTheWheels = new JoystickButton(m_flightcontroller, FlightController.X_THE_WHEELS);    
    btnXTheWheels.whileTrue(new XFactor(m_Drivetrain));


    JoystickButton btnWantACone = new JoystickButton(m_flightcontroller, FlightController.WANT_A_CONE);
    btnWantACone.onTrue(new LED_ColorSet(m_LEDLights, "cone"));
    btnWantACone.onFalse(new LED_ColorSet(m_LEDLights, "nothing"));

    JoystickButton btnWantACube = new JoystickButton(m_flightcontroller, FlightController.WANT_A_CUBE); 
    btnWantACube.onTrue(new LED_ColorSet(m_LEDLights, "cube"));
    btnWantACube.onFalse(new LED_ColorSet(m_LEDLights, "nothing"));

    // JoystickButton btnPickupIntake = new JoystickButton(m_flightcontroller, ControllerJoystick.PICKUP_INTAKE);
    JoystickButton btnRunPickup = new JoystickButton(m_controllerJoystick, ControllerJoystick.RUN_PICKUP);
    JoystickButton btnEjectPickup = new JoystickButton(m_controllerJoystick, ControllerJoystick.EJECT_PICKUP);

    JoystickButton btnExtendElevator = new JoystickButton(m_controllerJoystick, ControllerJoystick.EXTEND_ELEVATOR);
    JoystickButton btnRetractElevator = new JoystickButton(m_controllerJoystick, ControllerJoystick.RETRACT_ELEVATOR);

    // JoystickButton btnFlipperPickup = new JoystickButton(m_flightcontroller, ControllerJoystick.RUN_FLIPPER_INTAKE);
    JoystickButton btnWristUp = new JoystickButton(m_controllerJoystick, ControllerJoystick.WRIST_UP);
    JoystickButton btnWristDown = new JoystickButton(m_controllerJoystick, ControllerJoystick.WRIST_DOWN);    

    JoystickButton btnSkyhookBack = new JoystickButton(m_controllerJoystick, ControllerJoystick.SKYHOOK_REACHBACK);
    JoystickButton btnSkyhookForward = new JoystickButton(m_controllerJoystick, ControllerJoystick.SKYHOOK_REACHFORWARD);    

    JoystickButton btnSkyhookDriving = new JoystickButton(m_controllerJoystick, ControllerJoystick.SKYHOOK_DRIVING);    
    JoystickButton btnUprightCone = new JoystickButton(m_controllerJoystick, ControllerJoystick.UPRIGHTCONE_FRONT);    
    
    //JoystickButton btnLowFront = new JoystickButton(m_controllerJoystick, ControllerJoystick.SCORE_LOWFRONT);
    JoystickButton btnMidFront = new JoystickButton(m_controllerJoystick, ControllerJoystick.SCORE_MIDFRONT);
    JoystickButton btnTopFront = new JoystickButton(m_controllerJoystick, ControllerJoystick.SCORE_TOPFRONT);
    JoystickButton btnGroundPickupFront = new JoystickButton(m_controllerJoystick, ControllerJoystick.GROUNDPICKUP_FRONT);
    JoystickButton btnFeederPickupFront = new JoystickButton(m_controllerJoystick, ControllerJoystick.FEEDERPICKUP_FRONT);

    //JoystickButton btnScoreBackMid = new JoystickButton(m_controllerJoystick, ControllerJoystick.SCORE_BACK);
    //JoystickButton btnScoreBackHigh = new JoystickButton(m_controllerJoystick, ControllerJoystick.SCORE_BACKTIER3);

    //JoystickButton btnCameraToggle = new JoystickButton(m_controllerJoystick, ControllerJoystick.CAMERA_TOGGLE);
    
    btnResetDrivetrainOrientation.onTrue(new SetRobotOrientationOnField(m_Drivetrain, 0));//.andThen(m_Drivetrain::resetEncoders));


    btnRunPickup.onTrue(new SkyHook_RunIntake(m_skyHook, 0.5));
    btnRunPickup.onFalse(new SkyHook_RunIntake(m_skyHook, 0.06));

    btnEjectPickup.onTrue(new SkyHook_RunIntake(m_skyHook, -0.5));
    btnEjectPickup.onFalse(new SkyHook_RunIntake(m_skyHook, 0.0));
    
    btnExtendElevator.onTrue(new SkyHook_MoveElevator(m_skyHook, SkyHook.ExtensionPositions.EXTENDED)); //-200.0));
    btnExtendElevator.onFalse(new SkyHook_MoveElevator(m_skyHook, 0.0));
    btnRetractElevator.onTrue(new SkyHook_MoveElevator(m_skyHook, SkyHook.ExtensionPositions.RETRACTED)); // 100.0));// almost home
    btnRetractElevator.onFalse(new SkyHook_MoveElevator(m_skyHook,0.0));

    btnWristUp.onTrue(new SkyHook_MoveWrist(m_skyHook, WristPositions.BACKFOLDUP));
    btnWristUp.onFalse(new SkyHook_MoveWrist(m_skyHook, 0.0));

    btnWristDown.onTrue(new SkyHook_MoveWrist(m_skyHook, WristPositions.FRONTFOLDUP));
    btnWristDown.onFalse(new SkyHook_MoveWrist(m_skyHook, 0.0));

    btnSkyhookForward.whileTrue(new SkyHook_MoveArm(m_skyHook, ArmPositions.FULLFORWARD));
    btnSkyhookForward.onFalse(new SkyHook_MoveArm(m_skyHook, 0.0));

    btnSkyhookBack.onTrue(new SkyHook_MoveArm(m_skyHook, ArmPositions.FULLBACK));
    btnSkyhookBack.onFalse(new SkyHook_MoveArm(m_skyHook, 0.0));
    // // btnCameraToggle.whenPressed(m_Camera::changeCamera);
     //btnActivateLimelight.whenPressed(m_limelight::EnableVisionProcessing);

    btnSkyhookDriving.onTrue(new SkyHook_Scoring(m_skyHook, ArmPositions.DRIVING
    , ExtensionPositions.DRIVING, WristPositions.DRIVING));

    btnUprightCone.onTrue(new SkyHook_Scoring(m_skyHook, ArmPositions.UPRIGHTCONE_FRONT
        ,ExtensionPositions.UPRIGHTCONE_FRONT, WristPositions.UPRIGHTCONE_FRONT));

    btnMidFront.onTrue(new SkyHook_ScoringPlus(m_skyHook, ArmPositions.TIER2SCORE_FRONT
      , ExtensionPositions.TIER2SCORE_FRONT, WristPositions.TIER2SCORE_FRONT, m_controllerJoystick));

    btnTopFront.onTrue(new SkyHook_ScoringPlus(m_skyHook, ArmPositions.TIER3SCORE_FRONT
      , ExtensionPositions.TIER3SCORE_FRONT, WristPositions.TIER3SCORE_FRONT, m_controllerJoystick));

    //btnScoreBack.onTrue(new SkyHook_ScoreBack(m_skyHook));
    // btnScoreBackMid.onTrue(new SkyHook_ScoringPlus(m_skyHook, ArmPositions.TIER2SCORE_BACK
    // , ExtensionPositions.TIER2SCORE_BACK, WristPositions.TIER2SCORE_BACK, m_controllerJoystick));

    // btnScoreBackHigh.onTrue(new SkyHook_ScoringPlus(m_skyHook,ArmPositions.TIER3SCORE_BACK
    //   , ExtensionPositions.TIER3SCORE_BACK, WristPositions.TIER3SCORE_BACK, m_controllerJoystick));

    btnGroundPickupFront.onTrue(new SkyHook_Scoring(m_skyHook, ArmPositions.GROUNDPICKUP_FRONT
      , ExtensionPositions.GROUNDPICKUP_FRONT, WristPositions.GROUNDPICKUP_FRONT));

    btnFeederPickupFront.onTrue(new SkyHook_ScoringPlus(m_skyHook, ArmPositions.FEEDERPICKUP_FRONT
      , ExtensionPositions.FEEDERPICKUP_FRONT, WristPositions.FEEDERPICKUP_FRONT, m_controllerJoystick));
      //btnActivateLimelight.whenReleased(m_limelight::DisableVisionProcessing);
    }

      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
    //return ORIGgetAutonomousCommand();
    return m_chooser.getSelected();
    //return m_SimpleAutonCommand;
  }
  
//   public Command ORIGgetAutonomousCommand() {
//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(
//                 AutoConstants.kMaxSpeedMetersPerSecond,
//                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(DriveConstants.kDriveKinematics);

//     // An example trajectory to follow.  All units in meters.
//     Trajectory exampleTrajectory =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the origin facing the +X direction
//             new Pose2d(0, 0, new Rotation2d(0)),
//             // Pass through these two interior waypoints, making an 's' curve path
//             List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
//             // End 3 meters straight ahead of where we started, facing forward
//             new Pose2d(3, 0, new Rotation2d(0)),
//             config);

// //  return m_Drivetrain.createCommandForTrajectory(exampleTrajectory, true);

//     var thetaController =
//         new ProfiledPIDController(
//             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
// //SwerveControllerCommand x = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements)

// SwerveControllerCommand swerveControllerCommand =
//         new SwerveControllerCommand(
//             exampleTrajectory,
//             m_Drivetrain::getPose, // Functional interface to feed supplier
//             DriveConstants.kDriveKinematics,

//             // Position controllers
//             new PIDController(AutoConstants.kPXController, 0, 0),
//             new PIDController(AutoConstants.kPYController, 0, 0),
//             thetaController,
//             m_Drivetrain::setModuleStates,
//             m_Drivetrain);

//     // Reset odometry to the starting pose of the trajectory.
//     m_Drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     //return swerveControllerCommand;
//     return swerveControllerCommand.andThen(() -> m_Drivetrain.drive(0, 0, 0, false));

//   }


}
