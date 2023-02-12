// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDrive extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final DoubleSupplier m_forwardSpeed;
  private final DoubleSupplier m_rotationSpeed;

  /** Creates a new TeleopDrive. */
  public TeleopDrive(DriveSubsystem subsystem, DoubleSupplier fwd, DoubleSupplier rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = subsystem;
    m_forwardSpeed = fwd;
    m_rotationSpeed = rot;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.arcadeDrive(m_forwardSpeed.getAsDouble(), m_rotationSpeed.getAsDouble());
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
