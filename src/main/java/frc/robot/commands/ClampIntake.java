// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClampSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

public class ClampIntake extends CommandBase {
  private final ClampSubsystem m_clampSubsystem;
  private final XboxController m_controller;

  /** Creates a new ClampIntake. */
  public ClampIntake(ClampSubsystem clamp, XboxController controller) {
    m_clampSubsystem = clamp;
    m_controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clamp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.getAButtonPressed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clampSubsystem.setClampMotorSpeeds(kClampIntakeVelocity, kClampIntakeCurrentLimit);
    if(m_controller.getAButtonPressed()) 
      m_clampSubsystem.toggleSolenoid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clampSubsystem.setClampMotorSpeeds(0, kClampIntakeCurrentLimit);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
