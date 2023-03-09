package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleMK3 {

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 0.03; //15.0;
  private static final double kDriveI = 0.0; //0.01;
  private static final double kDriveD = 0.0; //0.1;
  private static final double kDriveF = 0.20; //0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  // CANCoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANCoder canCoder;
  private Rotation2d offset;

  public SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;
    this.offset = offset;
    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

    angleTalonFXConfiguration.slot0.kP = kAngleP;
    angleTalonFXConfiguration.slot0.kI = kAngleI;
    angleTalonFXConfiguration.slot0.kD = kAngleD;
    // Use the CANCoder as the remote sensor for the primary TalonFX PID
    angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
    angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    
    angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    angleMotor.configAllSettings(angleTalonFXConfiguration);
    angleMotor.setNeutralMode(NeutralMode.Brake); //not needed but nice to keep the robot stopped when you want it stopped
    angleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
    angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);

// angleMotor.configClosedloopRamp(0.5);
    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

    driveTalonFXConfiguration.slot0.kP = kDriveP;
    driveTalonFXConfiguration.slot0.kI = kDriveI;
    driveTalonFXConfiguration.slot0.kD = kDriveD;
    driveTalonFXConfiguration.slot0.kF = kDriveF;
 
    driveMotor.configAllSettings(driveTalonFXConfiguration);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.configOpenloopRamp(0.9);
    driveMotor.configClosedloopRamp(0.9);

    // set voltage compensation to 11 volts to smooth out battery variations
    driveMotor.configVoltageCompSaturation(11, 0);
    driveMotor.enableVoltageCompensation(true);

    // this line sets the ration of encoder pulses to actual distance travelled (in meters)
    driveMotor.configSelectedFeedbackCoefficient(DriveConstants.kEncoderDistancePerPulse);

    driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);
    canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10);
  }

  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition()); //include angle offset
  }
  public double getRawAngle() {
    return canCoder.getAbsolutePosition(); //include angle offset
  }
  //:)

  public void resetEncoders(){
    driveMotor.setSelectedSensorPosition(0);
  }

  public double getDistance() {
    //double x = driveMotor.getSelectedSensorPosition();
    return driveMotor.getSelectedSensorPosition() * Constants.DriveConstants.kEncoderDistancePerPulse;

  }
  public double getVelocity(){
    return driveMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse;
  }
    /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModulePosition getModulePosition() {
      double rawDistance = driveMotor.getSelectedSensorPosition();
      return new SwerveModulePosition(rawDistance * DriveConstants.kEncoderDistancePerPulse, getAngle());
  }
  
  public SwerveModuleState getState() {
    //  getSelectedSensorVelocity needs to be in Meters per Second.
    // Need to get conversion in kEncoderDistancePerPulse value of Constants
    double rawVelocity = driveMotor.getSelectedSensorVelocity();
    return new SwerveModuleState( rawVelocity * DriveConstants.kEncoderDistancePerPulse, getAngle());
  
  //  return new SwerveModuleState( driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_turningEncoder.get()));
  }

  public void setWheelLock(Rotation2d desiredAngle){
    Rotation2d currentRotation = getAngle();
    Rotation2d rotationDelta = desiredAngle.minus(currentRotation);
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;
    driveMotor.set(TalonFXControlMode.Velocity, 0);
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);
  }


  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    
    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;

      //double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    //double motorSpeed = feetPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
    double motorSpeed = state.speedMetersPerSecond / DriveConstants.kEncoderDistancePerPulse;
    //below is a line to comment out from step 5
    // original code had "PercentOutput"  Not sure why
   // driveMotor.set(TalonFXControlMode.PercentOutput, motorSpeed);

   driveMotor.set(TalonFXControlMode.Velocity, motorSpeed); //feetPerSecond / SwerveDrivetrain.kMaxSpeed);

     //below is a line to comment out from step 5
     if (motorSpeed==0)
     {
      angleMotor.set(TalonFXControlMode.PercentOutput, 0);
     }
     else
     {
      angleMotor.set(TalonFXControlMode.Position, desiredTicks);
     }
      // SmartDashboard.putNumber("driveMotorSpeed", motorSpeed);
      // SmartDashboard.putNumber("driveMotorMperS", state.speedMetersPerSecond);
  }
}
