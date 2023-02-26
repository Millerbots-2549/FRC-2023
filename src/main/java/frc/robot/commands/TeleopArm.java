// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.opencv.features2d.KAZE;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

public class TeleopArm extends CommandBase {

  private final ArmSubsystem m_armSubsystem;
  private final BooleanSupplier m_armIn;
  private final BooleanSupplier m_armOut;

  /** Creates a new ManualArmControl. */
  public TeleopArm(ArmSubsystem subsystem, BooleanSupplier in, BooleanSupplier out) {
    m_armSubsystem = subsystem;
    m_armIn = in;
    m_armOut = out;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.0;
    if(m_armIn.getAsBoolean()){
      speed = -kArmMotorAutoSpeed;
    }
    if(m_armOut.getAsBoolean()){
      speed = kArmMotorAutoSpeed;
    }
    m_armSubsystem.setArmMotorSpeed(speed);
    SmartDashboard.putNumber("arm speed", speed);
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
