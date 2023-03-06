// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BringArm;
import frc.robot.commands.ClampShoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;

import static frc.robot.Constants.ManipulatorConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHybridNode extends SequentialCommandGroup {
  /** Creates a new PlaceHybridNode. */
  public PlaceHybridNode(ArmSubsystem arm, ClampSubsystem clamp) {
    if(clamp.isClampInCubeMode())
      addCommands(
        new BringArm(arm, kArmBumperPosistion),
        new ClampShoot(clamp).withTimeout(kClampShootDuration),
        new WaitCommand(kPlaceCommandWaitTime),
        new BringArm(arm, kArmInsidePosition)
      );
    else
      addCommands(
        new BringArm(arm, kArmIntakePosition),
        new InstantCommand(clamp::toggleSolenoid),
        new WaitCommand(kPlaceCommandWaitTime),
        new BringArm(arm, kArmInsidePosition)
      ); 
  }
}
