// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DebugDriveSpeedBalancing extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private double m_motorVolts;

  /** Creates a new DebugDriveSpeedBalancing. */
  public DebugDriveSpeedBalancing(DriveSubsystem subsystem) {
    m_driveSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_motorVolts = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_driveSubsystem.getWheelSpeeds().leftMetersPerSecond == 0 && m_driveSubsystem.getWheelSpeeds().rightMetersPerSecond == 0)
      m_motorVolts += 0.005;
    m_driveSubsystem.tankDrive(m_motorVolts, m_motorVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
