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
import edu.wpi.first.wpilibj.PWM;
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
import frc.robot.Constants.FlightController;
import frc.robot.commands.SkyHook_MoveWrist;
import frc.robot.commands.SkyHook_RunIntake;
import frc.robot.commands.SkyHook_MoveArm;
import frc.robot.commands.SkyHook_MoveElevator;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ReplayFile;
//import frc.robot.commands.ReplayFile;
import frc.robot.commands.SetRobotOrientationOnField;
//import frc.robot.commands.Shoot;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DataRecorder;
//import frc.robot.subsystems.Intake;
import frc.robot.commands.SwerveDriveCommand;
//import frc.robot.commands.TransferToNextBar;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Limelight;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.VisionCamera;
import frc.robot.subsystems.SkyHook;
import frc.robot.subsystems.VisionCamera;
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
    Command SimpleTest = new SequentialCommandGroup(
      new SetRobotOrientationOnField(m_Drivetrain, 0.0),
      new ReplayFile(m_Drivetrain, m_skyHook, m_limelight, m_DataRecorder, "SimpleTest.csv")
      );

    // Command TwoBall_Center = new SequentialCommandGroup(
    //   new SetRobotOrientationOnField(m_Drivetrain, 0),
    //   new ReplayFile(m_Drivetrain, m_Intake, m_shooter, m_limelight, m_DataRecorder, "Center2Ball.csv"),
    //   new SetRobotOrientationOnField(m_Drivetrain, 160)   
    //   );
  
    // Command TwoBall_Left = new SequentialCommandGroup(
    //   new SetRobotOrientationOnField(m_Drivetrain, -0.02),
    //   new ReplayFile(m_Drivetrain, m_Intake, m_shooter, m_limelight, m_DataRecorder, "twoball-c.csv"),
    //   new SetRobotOrientationOnField(m_Drivetrain, -165)   
    //   );
    


    m_chooser = new SendableChooser<>();
    //m_chooser.addOption("Original", ORIGgetAutonomousCommand() );
    m_chooser.addOption("SimpleTest", SimpleTest);
    // m_chooser.addOption("2-Ball CENTER", TwoBall_Center);
    // m_chooser.addOption("2-Ball LEFT", TwoBall_Left);
    // m_chooser.addOption("3-Ball RIGHT", ThreeBall_Right);
    // m_chooser.addOption("3-Ball Run Forrest!", ThreeBall_RunForrest);
    // m_chooser.addOption("4-Ball RIGHT", FourBall_Right);


    //m_chooser.addOption("Barrel", new Barrel(m_drivetrain));
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  private void configureButtonBindings() {
    // // setup buttons

    // JoystickButton btnPickupIntake = new JoystickButton(m_flightcontroller, ControllerJoystick.PICKUP_INTAKE);
    JoystickButton btnRunPickup = new JoystickButton(m_controllerJoystick, ControllerJoystick.RUN_PICKUP);
    JoystickButton btnEjectPickup = new JoystickButton(m_controllerJoystick, ControllerJoystick.EJECT_PICKUP);

    JoystickButton btnExtendElevator = new JoystickButton(m_controllerJoystick, ControllerJoystick.EXTEND_ARM);
    JoystickButton btnRetractElevator = new JoystickButton(m_controllerJoystick, ControllerJoystick.RETRACT_ARM);

    // JoystickButton btnFlipperPickup = new JoystickButton(m_flightcontroller, ControllerJoystick.RUN_FLIPPER_INTAKE);
    JoystickButton btnWristUp = new JoystickButton(m_controllerJoystick, ControllerJoystick.WRIST_UP);
    JoystickButton btnWristDown = new JoystickButton(m_controllerJoystick, ControllerJoystick.WRIST_DOWN);    

    JoystickButton btnSkyhookBack = new JoystickButton(m_controllerJoystick, ControllerJoystick.SKYHOOK_REACHBACK);
    JoystickButton btnSkyhookHome = new JoystickButton(m_controllerJoystick, ControllerJoystick.SKYHOOK_GOHOME);    
    JoystickButton btnSkyhookForward = new JoystickButton(m_controllerJoystick, ControllerJoystick.SKYHOOK_REACHFORWARD);    

    JoystickButton btnAutoBalance = new JoystickButton(m_controllerJoystick, ControllerJoystick.AUTO_BALANCE);    
    btnAutoBalance.whileTrue(new AutoBalance(m_Drivetrain));

    // // T-Rex Buttons
    // JoystickButton btnTrexArmDown = new JoystickButton(m_controllerJoystick, ControllerJoystick.LOWER_TREX_ARMS);
    // JoystickButton btnTrexArmUp = new JoystickButton(m_controllerJoystick, ControllerJoystick.RAISE_TREX_ARMS);
    // JoystickButton btnTrexClose = new JoystickButton(m_controllerJoystick, ControllerJoystick.CLOSE_TREX_ARMS);
    // JoystickButton btnTrexOpen = new JoystickButton(m_controllerJoystick, ControllerJoystick.OPEN_TREX_ARMS);

    //JoystickButton btnCameraToggle = new JoystickButton(m_controllerJoystick, ControllerJoystick.CAMERA_TOGGLE);
    JoystickButton btnResetDrivetrainOrientation =  new JoystickButton(m_controllerJoystick, ControllerJoystick.REORIENT_ROBOT);
    
    btnResetDrivetrainOrientation.onTrue(new SetRobotOrientationOnField(m_Drivetrain, 0).andThen(m_Drivetrain::resetEncoders));

    //new JoystickButton(m_rightJoystick, RightJoystick.TOGGLE_LIMELIGHT).whenPressed(m_limelight::ToggleVisionProcessing, m_limelight);
    // btnShoot.whileHeld(new Shoot(m_shooter, m_limelight,
    //             () -> m_controllerJoystick.getRawButton(ControllerJoystick.FORCE_READY) ,
    //             () -> m_controllerJoystick.getRawButton(ControllerJoystick.TURBO_SHOT) ));
    // btnTrexArmDown.whileTrue(new DownTRexArms(m_tRexArms));
    // btnTrexArmUp.whileTrue(new UpTRexArms(m_tRexArms));
    // btnTrexClose.whileTrue(new CloseTRexArms(m_tRexArms));
    // btnTrexOpen.whileTrue(new OpenTRexArms(m_tRexArms));

    //btnFlipperDown.whileTrue(new SkyHook_MoveWrist(m_pancakeFlipper, PancakeFlipper.FlipperPosition.PICKUP));
    //btnFlipperUp.whileTrue(new SkyHook_MoveWrist(m_pancakeFlipper, PancakeFlipper.FlipperPosition.UP));

    btnRunPickup.onTrue(new SkyHook_RunIntake(m_skyHook, 0.5));
    btnRunPickup.onFalse(new SkyHook_RunIntake(m_skyHook, 0.0));

    btnEjectPickup.onTrue(new SkyHook_RunIntake(m_skyHook, -0.5));
    btnEjectPickup.onFalse(new SkyHook_RunIntake(m_skyHook, 0.0));
    
    btnExtendElevator.onTrue(new SkyHook_MoveElevator(m_skyHook, SkyHook.ExtensionPositions.EXTENDED)); //-200.0));
    btnExtendElevator.onFalse(new SkyHook_MoveElevator(m_skyHook, 0.0));
    btnRetractElevator.onTrue(new SkyHook_MoveElevator(m_skyHook, SkyHook.ExtensionPositions.RETRACTED)); // 100.0));// almost home
    btnRetractElevator.onFalse(new SkyHook_MoveElevator(m_skyHook,0.0));

    btnWristUp.onTrue(new SkyHook_MoveWrist(m_skyHook, WristPositions.EXTENDED));
    btnWristUp.onFalse(new SkyHook_MoveWrist(m_skyHook, 0.0));

    btnWristDown.onTrue(new SkyHook_MoveWrist(m_skyHook, WristPositions.RETRACTED));
    btnWristDown.onFalse(new SkyHook_MoveWrist(m_skyHook, 0.0));

    //btnSkyhookBack.onTrue(new SkyHook_MoveArm(m_skyHook, SkyHook.ArmFlip.BACK, false));
    btnSkyhookHome.onTrue(new SkyHook_MoveArm(m_skyHook, SkyHook.ArmPositions.STARTPOSITION, false));
    //btnSkyhookForward.onTrue(new SkyHook_MoveArm(m_skyHook, SkyHook.ArmFlip.FORWARD, false));
    btnSkyhookForward.whileTrue(new SkyHook_MoveArm(m_skyHook, -2.1, true));
    //btnSkyhookForward.onFalse(new SkyHook_MoveArm(m_skyHook, 0.0, false));

    btnSkyhookBack.whileTrue(new SkyHook_MoveArm(m_skyHook, 2.3, true));
    //btnSkyhookBack.onFalse(new SkyHook_MoveArm(m_skyHook, 0.0, false));
    // // btnCameraToggle.whenPressed(m_Camera::changeCamera);
     //btnActivateLimelight.whenPressed(m_limelight::EnableVisionProcessing);
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
