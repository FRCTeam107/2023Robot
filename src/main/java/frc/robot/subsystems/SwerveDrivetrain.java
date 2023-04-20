// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Motors;
import frc.robot.subsystems.DataRecorder.datapoint;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
public class SwerveDrivetrain extends SubsystemBase {


  // angle offsets for rotation motors
  // reducing the angle will adjust in counter-clockwise direction
  
  // //COMPETITION ROBOT SETTINGS
  public static double frontLeftOffset = 80;//89.9;
  public static double frontRightOffset = 159;//281.9;
  public static double backLeftOffset = 110;//116.9;
  public static double backRightOffset = 172;//304.8;

  // PRACTICE ROBOT SETTINGS
  // public static double frontLeftOffset = 115;//89.9;
  // public static double frontRightOffset = 21;//281.9;
  // public static double backLeftOffset = 130;//116.9;
  // public static double backRightOffset = 190;//304.8;

// field offset angle
  private double m_fieldOffsetAngle = 0;
  
  public static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private SwerveDriveOdometry m_odometry ;

  //  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
  //      m_gyro.getRotation2d(), getModulePositions());


   private SwerveModuleMK3 m_frontLeft, m_frontRight, m_rearLeft, m_rearRight;
   private SwerveModuleMK3[] modules;

   public SwerveDrivetrain(double fieldOffsetAngle) {
    // gyro.reset(); 
    setFieldOffsetAngle(fieldOffsetAngle);  // at time of initializtion, assume a zero offset
  
    m_frontLeft = new SwerveModuleMK3(new TalonFX(Motors.frontLeftDriveId), new TalonFX(Motors.frontLeftSteerId), new CANCoder(Motors.frontLeftCANCoderId), Rotation2d.fromDegrees(frontLeftOffset));
    m_frontRight = new SwerveModuleMK3(new TalonFX(Motors.frontRightDriveId), new TalonFX(Motors.frontRightSteerId), new CANCoder(Motors.frontRightCANCoderId), Rotation2d.fromDegrees(frontRightOffset));
    m_rearLeft = new SwerveModuleMK3(new TalonFX(Motors.backLeftDriveId), new TalonFX(Motors.backLeftSteerId), new CANCoder(Motors.backLeftCANCoderId), Rotation2d.fromDegrees(backLeftOffset));
    m_rearRight = new SwerveModuleMK3(new TalonFX(Motors.backRightDriveId), new TalonFX(Motors.backRightSteerId), new CANCoder(Motors.backRightCANCoderId), Rotation2d.fromDegrees(backRightOffset));
   
    modules = new SwerveModuleMK3[] {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, 
       m_gyro.getRotation2d(), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] x = new SwerveModulePosition[] { m_frontLeft.getModulePosition(), 
    m_frontRight.getModulePosition(), m_rearLeft.getModulePosition(), m_rearRight.getModulePosition()};
    return x;
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param calibrateGyro button to recalibrate the gyro offset
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
    // if(calibrateGyro){
    //   //m_gyro.reset(); //recalibrates gyro offset
    //   zeroHeading(0); // assume manual re-calibrating pointing straight 'north' on field
    // }
    SmartDashboard.putNumber("dataRecorder." + datapoint.Drive_X, xSpeed);
    SmartDashboard.putNumber("dataRecorder." + datapoint.Drive_Y, ySpeed);
    SmartDashboard.putNumber("dataRecorder." + datapoint.Drive_Z, rot);
    SmartDashboard.putNumber("dataRecorder." + datapoint.GyroAngle, m_gyro.getAngle());

    // SmartDashboard.putNumber("dataRecorder." + datapoint.frontLeftDistance, m_frontLeft.getDistance());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.frontLeftVelocity, m_frontLeft.getVelocity());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.frontRightDistance, m_frontRight.getDistance());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.frontRightVelocity, m_frontRight.getVelocity());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.rearLeftDistance, m_rearLeft.getDistance());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.rearLeftVelocity, m_rearLeft.getVelocity());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.rearRightDistance, m_rearRight.getDistance());
    // SmartDashboard.putNumber("dataRecorder." + datapoint.rearRightVelocity, m_rearRight.getVelocity());
  
    SwerveModuleState[] states =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle()))
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
    //SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    // SmartDashboard.putNumber("gyro Angle", m_gyro.getAngle());
    // SmartDashboard.putNumber("gyro Roll", m_gyro.getRoll());
    // SmartDashboard.putNumber("gyro Pitch", m_gyro.getPitch());

    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      //below is a line to comment out from step 5
      module.setDesiredState(state);
    }
  }
  

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Front-left", m_frontLeft.getRawAngle());
    // SmartDashboard.putNumber("Front-right", m_frontRight.getRawAngle());
    // SmartDashboard.putNumber("Rear-left", m_rearLeft.getRawAngle());
    // SmartDashboard.putNumber("Rear-right", m_rearRight.getRawAngle());
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(  m_gyro.getRotation2d(), getModulePositions()      );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

   /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
    desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getModulePositions() ,pose);
  }
  
    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
      m_frontLeft.resetEncoders();
      m_rearLeft.resetEncoders();
      m_frontRight.resetEncoders();
      m_rearRight.resetEncoders();
    }
  
    /** Zeroes the heading of the robot. Based on fieldOffsetAngle */
    public void setFieldOffsetAngle(double fieldOffsetAngle) {
      m_gyro.reset();
      m_gyro.setAngleAdjustment(fieldOffsetAngle);
      m_fieldOffsetAngle = fieldOffsetAngle;
    }

    public double getFieldOffsetAngle(){
      return m_fieldOffsetAngle;
    }

    public void xFormat(){
      m_frontLeft.setWheelLock(Rotation2d.fromDegrees(35));
      m_frontRight.setWheelLock(Rotation2d.fromDegrees(-70));
      m_rearLeft.setWheelLock(Rotation2d.fromDegrees(-35));
      m_rearRight.setWheelLock(Rotation2d.fromDegrees(70));   
    }
  
    /**
     * Returns the angle heading of the robot.
    */
    public double getAngle() {
      return m_gyro.getAngle();
    }
    public double getPitch() {
      return m_gyro.getPitch();
    }
    public double getRoll() {
      return m_gyro.getRoll();
    }
    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public float getYaw() {
      return m_gyro.getYaw();
    }

    public double getHeading() {
      return m_gyro.getRotation2d().getDegrees();
    }

    public double[] getDistances(){
      double[] retVal ={m_frontLeft.getDistance(), m_frontRight.getDistance(),
              m_rearLeft.getDistance(), m_rearRight.getDistance()};
      return retVal;
    }
  
    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
      return m_gyro.getRate(); // * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }


   /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {

    var thetaController =
    new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //SwerveControllerCommand x = new SwerveControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, outputModuleStates, requirements)

    // sample code utilized the smart speed controllers instead of on-board code in RoboRio 
    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            this::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            this::setModuleStates,
            this);

    if (initPose) {
      // Reset odometry to the starting pose of the trajectory.
      var reset =  new InstantCommand(() -> this.resetOdometry(trajectory.getInitialPose()));
      return reset.andThen(swerveControllerCommand.andThen(() -> this.drive(0, 0, 0, false)));
    }
    else {
      return swerveControllerCommand.andThen(() -> this.drive(0, 0, 0, false));
    }
  }
  

    protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
      return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve("paths/" + trajectoryName + ".wpilib.json"));
        // Filesystem.getDeployDirectory().toPath().resolve(Paths.get("output", trajectoryName + ".wpilib.json")));
    }
    
    public Trajectory loadTrajectoryFromFile(String filename) {
      try {
        return loadTrajectory(filename);
      } catch (IOException e) {
        DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
        return new Trajectory();
      }
    }

  }
