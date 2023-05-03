// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.DriveConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBlueRightTaxi extends SequentialCommandGroup {
  /** Creates a new AutoBlueRightTaxi. */
  public AutoBlueRightTaxi(ArmSubsystem arm, ClampSubsystem clamp, ElevatorSubsystem elevator, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoPlaceCubeHigh(arm, clamp, elevator),
      new DriveStraight(drive, -kDriveAutoSpeed, 180).until(() -> drive.getPose().getX() > 2),
      new RunCommand(() -> drive.tankDrive(kDriveAutoSpeed, -kDriveAutoSpeed), drive).until(() -> drive.getPose().getRotation().getDegrees() < 163),
      new DriveStraight(drive, -kDriveAutoSpeed, 163).until(() -> drive.getPose().getY() < 0.725),
      new RunCommand(() -> drive.tankDrive(-kDriveAutoSpeed, kRightMotor1Port), drive).until(() -> drive.getPose().getRotation().getDegrees() > 180 || drive.getPose().getRotation().getDegrees() < 0),
      new DriveStraight(drive, -kDriveAutoSpeed, 180).until(() -> drive.getPose().getX() > 4.5)
    );
  }
}