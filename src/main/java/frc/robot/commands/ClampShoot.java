// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ManipulatorConstants.*;
import frc.robot.subsystems.ClampSubsystem;

public class ClampShoot extends CommandBase {
  private final ClampSubsystem m_clampSubsystem;
  
  /** Creates a new ClampShoot. */
  public ClampShoot(ClampSubsystem clamp) {
    m_clampSubsystem = clamp;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clampSubsystem.setClampMotorSpeeds(kClampShootVelocity, kClampShootVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clampSubsystem.setClampMotorSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
