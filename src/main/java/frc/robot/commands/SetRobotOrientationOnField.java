// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetRobotOrientationOnField extends InstantCommand {
  private final SwerveDrivetrain m_Drivetrain;
  private final double fieldOffsetAngle;
  
  public SetRobotOrientationOnField(SwerveDrivetrain _Drivetrain, double _fieldOffsetAngle) {
    m_Drivetrain = _Drivetrain;
    fieldOffsetAngle = _fieldOffsetAngle;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Drivetrain.setFieldOffsetAngle(fieldOffsetAngle);
    m_Drivetrain.resetEncoders();
  }
}
