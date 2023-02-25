// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ManipulatorConstants.*;
import frc.robot.subsystems.ElevatorSubsystem;

public class BringElevatorUp extends CommandBase {
  private final ElevatorSubsystem m_elevatorSubsystem;

  /** Creates a new BringElevatorUp. */
  public BringElevatorUp(ElevatorSubsystem subsystem) {
    m_elevatorSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevatorSubsystem.setElevatorMotorSpeed(-kElevatorMotorAutoSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setElevatorMotorSpeed(0);
    m_elevatorSubsystem.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_elevatorSubsystem.getElevatorMotorCurrent()) > kElevatorMotorStallCurrent;
  }
}
