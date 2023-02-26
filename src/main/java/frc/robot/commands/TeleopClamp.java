// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClampSubsystem;

public class TeleopClamp extends CommandBase {
  private final ClampSubsystem m_clampSubsystem;
  private final BooleanSupplier m_toggleSolenoidSupplier;
  private final DoubleSupplier m_clampMotorSpeeds;
  private final BooleanSupplier m_clampMotorReverse;

  /** Creates a new TeleopClamp. */
  public TeleopClamp(ClampSubsystem subsystem, BooleanSupplier toggleSolenoids, DoubleSupplier motorSpeeds, BooleanSupplier motorReverse) {
    m_clampSubsystem = subsystem;
    m_toggleSolenoidSupplier = toggleSolenoids;
    m_clampMotorSpeeds = motorSpeeds;
    m_clampMotorReverse = motorReverse;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_clampMotorSpeeds.getAsDouble();
    if(m_clampMotorReverse.getAsBoolean()){
      speed *= -1;
    }
    m_clampSubsystem.setClampMotorSpeeds(speed, speed);
    if(m_toggleSolenoidSupplier.getAsBoolean()){
      m_clampSubsystem.toggleSolenoid();
    }
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
