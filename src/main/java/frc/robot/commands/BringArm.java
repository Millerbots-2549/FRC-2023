// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ManipulatorConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BringArm extends ProfiledPIDCommand {
  /** Creates a new BringArm. */
  public BringArm(ArmSubsystem arm, double kSetpoint) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            kArmMotorKP,
            0.1,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(kArmMaxVelocity, kArmMaxAcceleration)),
        // This should return the measurement
        arm::getEncoderDistance,
        // This should return the goal (can also be a constant)
        () -> kSetpoint,
        // This uses the output
        (output, setpoint) -> {
          arm.setMotorVolts(output, setpoint.velocity);
          SmartDashboard.putNumber("output volts", output);
          SmartDashboard.putNumber("output velocity", setpoint.velocity);
          SmartDashboard.putNumber("setpoint", kSetpoint);
          SmartDashboard.putNumber("output position", setpoint.position);
        });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(kArmPositionTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
