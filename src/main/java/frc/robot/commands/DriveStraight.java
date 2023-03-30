// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStraight extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final double kSpeed;
  private final double kHeading;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveSubsystem drive, double speed, double heading) {
    m_driveSubsystem = drive;
    kSpeed = speed;
    kHeading = heading;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = (m_driveSubsystem.getPose().getRotation().getDegrees() < -90) ? m_driveSubsystem.getPose().getRotation().getDegrees() + 360 : m_driveSubsystem.getPose().getRotation().getDegrees();
    error -= kHeading;
    double leftDriveSpeed = kSpeed + error*0.01;
    double rightDriveSpeed = kSpeed - error*0.01;
    m_driveSubsystem.tankDrive(leftDriveSpeed, rightDriveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
