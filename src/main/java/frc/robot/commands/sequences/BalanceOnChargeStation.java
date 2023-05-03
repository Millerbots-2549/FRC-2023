// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceOnChargeStation extends SequentialCommandGroup {
  /** Creates a new BalanceOnChargeStation. */
  public BalanceOnChargeStation(DriveSubsystem subsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> subsystem.resetGyro(), subsystem),
      new RunCommand(() -> subsystem.tankDrive(kBalanceWheelSpeedHigh, kBalanceWheelSpeedHigh), subsystem).until(() -> subsystem.getRobotPitch() < kBalanceAirPitch),
      new RunCommand(() -> {
        subsystem.tankDrive(-0.52, -0.52);
      }, subsystem).until(() -> subsystem.getRobotPitch() > kBalanceDropPitch),
     
      new RunCommand(() -> {

        if(Math.abs(subsystem.getRobotPitch()) > kBalanceHomeostasisPitch)
          subsystem.tankDrive(0.46*Math.signum(subsystem.getRobotPitch()), 0.46*Math.signum(subsystem.getRobotPitch()));
        else
          subsystem.tankDrive(0.0, 0.0);
      }, subsystem)
    );
  }
}
