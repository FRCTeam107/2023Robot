// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.CloseTRexArms;
import frc.robot.commands.DownTRexArms;
import frc.robot.commands.OpenTRexArms;
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
import frc.robot.subsystems.TRexArms;
import frc.robot.subsystems.PancakeFlipper;
//import frc.robot.subsystems.VisionCamera;
import frc.robot.subsystems.SkyHook;
import frc.robot.subsystems.VisionCamera;
//import frc.robot.commands.ClimberResetToHome;
//import frc.robot.commands.DismountFirstBar;
//import frc.robot.commands.ReachForTheBar;
//import frc.robot.commands.PullUpOntoTalonHooks;
//import frc.robot.commands.RunClimberManually;

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
  //private final Shooter m_shooter;
  //private final Intake m_Intake;
  //private final Climber m_climber;
  private final TRexArms m_tRexArms;
  private final LEDLights m_LEDLights;
  //private final VisionCamera m_Camera;
  private final Limelight m_limelight;
  // private final PWM m_limit;
  private final PancakeFlipper m_flipArm;
  // private final PWM m_limit;
  private final SkyHook m_skyHook;
  
  public DataRecorder m_DataRecorder = new DataRecorder();

  // private final Compressor m_compressor;

  //private final XboxController controller = new XboxController(0);
  //private final Joystick controller = new Joystick(0);

      // A chooser for autonomous commands
  private final SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_flightcontroller = new Joystick(Constants.UsbPorts.LEFT_STICK);
    m_flightcontroller.setXChannel(FlightController.DRIVE_X_AXIS);
    m_flightcontroller.setYChannel(FlightController.DRIVE_Y_AXIS);
    m_flightcontroller.setZChannel(FlightController.DRIVE_Z_AXIS);
    // m_limit = new PWM(0);
    m_skyHook = new SkyHook();

    // m_rightJoystick = new Joystick(Constants.UsbPorts.RIGHT_STICK);
    m_controllerJoystick = new Joystick(Constants.UsbPorts.CONTROLLER_STICK);
     m_LEDLights = new LEDLights();
    m_Drivetrain  = new SwerveDrivetrain(0);  // begin assuming no field offset angle of robot (facing straight "north")
    //m_Intake = new Intake ();
    //m_shooter = new Shooter();
    //m_climber = new Climber();
    //m_Camera = new VisionCamera();
     m_limelight = new Limelight();

    // m_DataRecorder = new DataRecorder();

    // m_motor = new TRexArms();

    m_tRexArms = new TRexArms();
    m_flipArm = new PancakeFlipper();
   
     m_Drivetrain.setDefaultCommand(new SwerveDriveCommand(m_Drivetrain, m_flightcontroller, m_limelight, m_LEDLights));
    
    configureButtonBindings();

    // Add commands to the autonomous command chooser
    // Command TwoBall_Right = new SequentialCommandGroup(
    //   new SetRobotOrientationOnField(m_Drivetrain, 82.54),
    //   new ReplayFile(m_Drivetrain, m_Intake, m_shooter, m_limelight, m_DataRecorder, "twoball-right.csv")
    //   );

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
    
    // Command ThreeBall_Right = new SequentialCommandGroup(
    //     new SetRobotOrientationOnField(m_Drivetrain, 82.54),
    //     new ReplayFile(m_Drivetrain, m_Intake, m_shooter, m_limelight, m_DataRecorder, "3Ball-1FAST.csv")
    //     // new ParallelCommandGroup(
    //     //   new ClimberResetToHome(m_climber),
    //     //   new ReplayFile(m_Drivetrain, m_Intake, m_shooter, m_DataRecorder, "3Ball-1FAST.csv")
    //     //   )   
    //     );
      
    // Command ThreeBall_RunForrest = new SequentialCommandGroup(
    //       new SetRobotOrientationOnField(m_Drivetrain, 82.54),
    //       new ReplayFile(m_Drivetrain, m_Intake, m_shooter, m_limelight, m_DataRecorder, "3Ball-RUN.csv")
    //       );

    m_chooser = new SendableChooser<>();
    //m_chooser.addOption("Original", ORIGgetAutonomousCommand() );
    // m_chooser.addOption("2-Ball RIGHT", TwoBall_Right);
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
    // JoystickButton btnShoot = new JoystickButton(m_controllerJoystick, ControllerJoystick.SHOOT);
    // JoystickButton btnIntakeDown = new JoystickButton(m_controllerJoystick, ControllerJoystick.PICKUP_DOWN);
    // JoystickButton btnIntakeUp = new JoystickButton(m_controllerJoystick, ControllerJoystick.PICKUP_UP);
    // JoystickButton btnPickupEject = new JoystickButton(m_controllerJoystick, ControllerJoystick.PICKUP_EJECT);
    // JoystickButton btnPickupIntake = new JoystickButton(m_flightcontroller, ControllerJoystick.PICKUP_INTAKE);
    // JoystickButton btnFlipperPickup = new JoystickButton(m_flightcontroller, ControllerJoystick.RUN_FLIPPER_INTAKE);
    // JoystickButton btnFlipperUp = new JoystickButton(m_flightcontroller, ControllerJoystick.FLIPPER_UP);
    // JoystickButton btnFlipperDown = new JoystickButton(m_flightcontroller, ControllerJoystick.FLIPPER_DOWN);
    // JoystickButton btnResetEncoder = new JoystickButton(m_flightcontroller, ControllerJoystick.RESET_ENCODER);
    // JoystickButton btnArmExtend = new JoystickButton(m_flightcontroller, ControllerJoystick.ARM_EXTEND);
    // JoystickButton btnArmRetract = new JoystickButton(m_flightcontroller, ControllerJoystick.ARM_RETRACT);

    JoystickButton btnSqueeze = new JoystickButton(m_controllerJoystick, 1);
    JoystickButton btnReleeve = new JoystickButton(m_controllerJoystick, 2);

    JoystickButton btnIntake = new JoystickButton(m_controllerJoystick, 3);
    JoystickButton btnStop = new JoystickButton(m_controllerJoystick, 5);
    JoystickButton btnPoop = new JoystickButton(m_controllerJoystick, 4);
    // JoystickButton btnClimbGrabNext = new JoystickButton(m_controllerJoystick, ControllerJoystick.CLIMBER_GRABNEXTBAR);
    // JoystickButton btnClimbDismount = new JoystickButton(m_controllerJoystick, ControllerJoystick.CLIMBER_DISMOUNT);

    // T-Rex Buttons
    JoystickButton btnSlapTRexArms = new JoystickButton(m_controllerJoystick, ControllerJoystick.SLAP_TREX_ARMS);
    JoystickButton btnClapTRexArms = new JoystickButton(m_controllerJoystick, ControllerJoystick.CLAP_TREX_ARMS);

    //JoystickButton btnCameraToggle = new JoystickButton(m_controllerJoystick, ControllerJoystick.CAMERA_TOGGLE);
    // JoystickButton btnResetDrivetrainOrientation =  new JoystickButton(m_controllerJoystick, ControllerJoystick.REORIENT_ROBOT);
    // JoystickButton btnClimbManualMode = new JoystickButton(m_controllerJoystick, ControllerJoystick.CLIMBER_MANUAL);
    // JoystickButton btnActivateLimelight = new JoystickButton(m_flightcontroller, FlightController.ACTIVATE_LIMELIGHT);

    // new JoystickButton(m_controllerJoystick,11).whileHeld(m_Intake::allowAdditionalMovement);
    
    // btnResetDrivetrainOrientation.whenPressed(new SetRobotOrientationOnField(m_Drivetrain, 0).andThen(m_Drivetrain::resetEncoders));

    //new JoystickButton(m_rightJoystick, RightJoystick.TOGGLE_LIMELIGHT).whenPressed(m_limelight::ToggleVisionProcessing, m_limelight);
    // btnShoot.whileHeld(new Shoot(m_shooter, m_limelight,
    //             () -> m_controllerJoystick.getRawButton(ControllerJoystick.FORCE_READY) ,
    //             () -> m_controllerJoystick.getRawButton(ControllerJoystick.TURBO_SHOT) ));
   


    // btnIntakeDown.whenPressed(m_Intake::extendArm);
    // btnIntakeUp.whenPressed(m_Intake::retractArm);

    // btnPickupEject.whileHeld(m_Intake::HeimlichManeuver);
    // btnPickupEject.whenReleased(m_Intake::StopIntake);
    btnSlapTRexArms.whileTrue(new DownTRexArms(m_tRexArms));
    btnClapTRexArms.whileTrue(new CloseTRexArms(m_tRexArms));
        
    
    // btnPickupIntake.whenPressed(m_motor::StartIntake);
    // btnPickupIntake.whenReleased(m_motor::StopIntake);
    btnIntake.whenPressed(m_flipArm::Pickup);
    btnStop.whenPressed(m_flipArm::StopPickup);
    btnPoop.whenPressed(m_flipArm::Poop);

    // btnPickupIntake.whenPressed(m_TRexArms::StartIntake);
    // btnPickupIntake.whenReleased(m_TRexArms::StopIntake);

    // btnResetEncoder.whenPressed(m_TRexArms::ZeroEncoder);
    // btnArmExtend.whenPressed(m_TRexArms::ArmExtend);
    // btnArmRetract.whenPressed(m_TRexArms::ArmRetract);

    btnSqueeze.whenPressed(m_skyHook::squeeze);
    btnReleeve.whenPressed(m_skyHook::releeve);

    // CONTROLLER'S JOYSTICK BUTTONS
    // btnClimbFirstBar.whileHeld(new ReachForTheBar(m_climber, m_LEDLights));
    // btnClimbPullup.whileHeld(new PullUpOntoTalonHooks(m_climber, m_LEDLights));
    // btnClimbGrabNext.whileHeld(new TransferToNextBar(m_climber, m_LEDLights,
    //        () -> m_controllerJoystick.getRawButton(ControllerJoystick.FORCE_READY) ));
    // btnClimbDismount.whileHeld(new DismountFirstBar(m_climber));

    // btnClimbManualMode.whileHeld(new RunClimberManually(m_climber, m_controllerJoystick));

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
