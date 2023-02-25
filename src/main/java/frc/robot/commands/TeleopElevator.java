// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class TeleopElevator extends CommandBase {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DoubleSupplier m_elevatorSpeedSupplier;

  /** Creates a new TeleopElevator. */
  public TeleopElevator(ElevatorSubsystem subsystem, DoubleSupplier speed) {
    m_elevatorSubsystem = subsystem;
    m_elevatorSpeedSupplier = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setElevatorMotorSpeed(m_elevatorSpeedSupplier.getAsDouble());
    SmartDashboard.putNumber("elevator speed", m_elevatorSpeedSupplier.getAsDouble());
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
