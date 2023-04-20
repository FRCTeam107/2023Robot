/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public class UsbPorts{
                public static final int FlightController = 0;
                public static final int CONTROLLER_STICK = 1;
                //public static final int CAMERA_PORT = 0;
        }

        //JOYSTICKS
        public class FlightController{
                public static final int DRIVE_X_AXIS = 3;
                public static final int DRIVE_Y_AXIS = 4;
                public static final int DRIVE_Z_AXIS = 0;
                
                public static final int ACTIVATE_LIMELIGHT = 1;
                static final int REORIENT_ROBOT = 16;
                static final int SPIN_180 = 13;
                public static final int AUTO_BALANCE = 6;

                static final int WANT_A_CONE = 4;
                static final int WANT_A_CUBE = 5;

                //public static final int CLIMBER_MANUAL = 4;
                public static final int AIM_BY_LIMELIGHT = 14;
                public static final int X_THE_WHEELS = 13;
                // // public static final int JOYSTICK_FIVE = 5;
                // //public static final int TURRET_LEFT = 4;
                // public static final int GREEN_ZONE = 11;
                // public static final int BLUE_ZONE = 12;
                // public static final int SYNC_TURRET = 7;
                // // public static final int JOYSTICK_EIGHT = 8;
                // public static final int RED_ZONE = 15;
                // public static final int YELLOW_ZONE = 16;
                // // public static final int ORCHESTRA = 13;
        }

        // public class RightJoystick{
        
        // }

        public class ControllerJoystick{
                //public static final int SHOOT = 1;
                public static final int EJECT_PICKUP = 1;
                static final int RUN_PICKUP = 2;

                static final int SKYHOOK_DRIVING = 3;
                public static final int GROUNDPICKUP_FRONT = 4;

                public static final int SCORE_TOPFRONT = 5;
                public static final int SCORE_MIDFRONT = 6;
                public static final int FEEDERPICKUP_FRONT = 7;

                public static final int UPRIGHTCONE_FRONT = 8;

                public static final int SCORE_BACK = 9;
                public static final int SCORE_BACKTIER3 = 10;

                static final int SKYHOOK_REACHBACK = 13;
                static final int SKYHOOK_REACHFORWARD = 14;

                public static final int WRIST_UP = 11;
                public static final int WRIST_DOWN = 16;

                // public static final int SKYHOOK_EXTEND = 13;
                // public static final int SKYHOOK_RETRACT = 14;
                static final int EXTEND_ELEVATOR = 12;
                static final int RETRACT_ELEVATOR = 15;
  
                //public static final int SCORE_LOWFRONT = 11; 
        }

        //MOTORS
        public class Motors{
               //put your can Id's here!
                public static final int frontLeftDriveId = 1; 
                public static final int frontLeftCANCoderId = 2; 
                public static final int frontLeftSteerId = 3;

                public static final int frontRightDriveId = 4; 
                public static final int frontRightCANCoderId = 5; 
                public static final int frontRightSteerId = 6; 

                public static final int backLeftDriveId = 7; 
                public static final int backLeftCANCoderId = 8; 
                public static final int backLeftSteerId = 9;

                public static final int backRightDriveId = 10; 
                public static final int backRightCANCoderId = 11; 
                public static final int backRightSteerId = 12;

                public static final int SKYHOOK_WRIST = 13;
                public static final int SKYHOOK_INTAKE = 14;
                public static final int SKYHOOK_EXTENDER = 16;
                public static final int SKYHOOK_ARM = 17;
                public static final int SKYHOOK_ARM2 = 18;

                public static final int SKYHOOK_CANIFIER = 1;
        }

        // DIO Ports
        // public class DIOPorts{
        //    public static final int TALONHOOK_LEFT = 0;
        //    public static final int TALONHOOK_RIGHT = 1;
        // }

        // Trajectory / Pathweaver constants
        public static final class DriveConstants {
               // public static final double kTrackwidthMeters = 1.7;// 1.685; // Units.inchesToMeters(27.0);
                // public static final DifferentialDriveKinematics kDriveKinematics =
                //     new DifferentialDriveKinematics(kTrackwidthMeters);
            
                public static final int kEncoderCPR = 2048; // 4096;
                public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0); // 0.1524;
                private static final double kWheelGearing = 8.16;  // gearing from motor to swerve drive
                public static final double kEncoderDistancePerPulse = 0.25 *
                      (kWheelDiameterMeters * Math.PI) / (double) ( kEncoderCPR / kWheelGearing) ;
                //Units.inchesToMeters(3.0) * Math.PI / 16592;  // actual pulses per rotation measured
                
                  // Assumes the encoders are directly mounted on the wheel shafts
                    //(kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;
            
                    // Distance between front and back wheels on robot
                private static double kWheelBase = Units.inchesToMeters(18);
                private static double kTrackWidth = Units.inchesToMeters(20);

                public static final SwerveDriveKinematics kDriveKinematics =
                        new SwerveDriveKinematics(
                                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
                public static final double kMaxSpeedMetersPerSecond = 1.05; //0.85;//0.88;//0.85;//Units.feetToMeters(6);

  //these are limits you can change!!!
               // public static final double kMaxSpeed = Units.feetToMeters(13);//(13.6); // 20 feet per second
                public static final double kMaxAngularSpeed = Math.PI / 1.5;// / 2; // 1/2 rotation per second
              // public static double feildCalibration = 0;
                
                    // These characterization values MUST be determined either experimentally or theoretically
                // for *your* robot's drive.
                // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
                // values for your robot.
                public static final double ksVolts = 0.519; //0.596;
                public static final double kvVoltSecondsPerMeter = 4.13; //4.13; //2.09; //0.452;
                public static final double kaVoltSecondsSquaredPerMeter = 0.255; //0.155; // 0.0255;

                // Example value only - as above, this must be tuned for your drive!
                // public static final double kPDriveVel = 0.965;
                public static final double kPDriveVel = 1.62; //0.8;
              }
              

              public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 1;
                public static final double kMaxAccelerationMetersPerSecondSquared = 1;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
            
                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;
            
                // Constraint for the motion profilied robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                    new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
              }
}
