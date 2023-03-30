// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ZeroArm extends CommandBase {
  private final ArmSubsystem m_armSubsystem;
   
  /** Creates a new ZeroArm. */
  public ZeroArm(ArmSubsystem arm) {
    m_armSubsystem = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setMotorSpeed(kArmCalibrationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.resetEncoder();
    m_armSubsystem.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armSubsystem.getCurrent() > kArmMotorStallCurrent;
  }
}
