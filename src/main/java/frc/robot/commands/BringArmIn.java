// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.ArmSubsystem;

public class BringArmIn extends CommandBase {

  private final ArmSubsystem m_armSubsystem;
  
  /** Creates a new BringArmIn. */
  public BringArmIn(ArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setArmMotorSpeed(-ManipulatorConstants.kArmMotorAutoSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setArmMotorSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_armSubsystem.getArmMotorCurrent()) > ManipulatorConstants.kArmMotorStallCurrent;
  }
}
