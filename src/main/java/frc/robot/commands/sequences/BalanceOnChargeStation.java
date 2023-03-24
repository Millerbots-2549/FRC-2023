// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveStraight;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceOnChargeStation extends SequentialCommandGroup {
  /** Creates a new BalanceOnChargeStation. */
  public BalanceOnChargeStation(DriveSubsystem subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveStraight(0, true, subsystem, subsystem.getHeading()).until(() -> subsystem.getRobotPitch() > DriveConstants.kBalanceAirPitch),
      new DriveStraight(DriveConstants.kBalanceSpeedMetersPerSecond, isFinished(), subsystem, subsystem.getHeading()).until(() -> subsystem.getRobotPitch() < DriveConstants.kBalanceDropPitch),
      new InstantCommand(() -> subsystem.tankDriveVolts(0, 0))
    );
  }
}
