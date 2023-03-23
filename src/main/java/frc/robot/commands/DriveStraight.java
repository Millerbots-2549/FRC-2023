// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveStraight extends SequentialCommandGroup {
  /** Creates a new DriveStraight. */
  public DriveStraight(double speed, boolean backwards, DriveSubsystem subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RamseteCommand(TrajectoryGenerator.generateTrajectory(subsystem.getPose(), List.of(new Translation2d(subsystem.getPose().getX() + 100*Math.cos(subsystem.getPose().getRotation().getDegrees())*((backwards) ? -1 : 1), subsystem.getPose().getY() + 100*Math.sin(subsystem.getPose().getRotation().getDegrees())*((backwards) ? -1 : 1))), subsystem.getPose(), subsystem.getTrajectoryConfig(speed)), 
      subsystem::getPose, 
      new RamseteController(),
      new SimpleMotorFeedforward(speed, speed),
      subsystem.getKinematics(), 
      subsystem::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      (a, b) -> subsystem.tankDriveVolts(a, b),
      subsystem)
    );
  }
}
