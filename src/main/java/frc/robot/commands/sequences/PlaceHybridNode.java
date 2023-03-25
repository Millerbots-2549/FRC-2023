// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BringArm;
import frc.robot.commands.BringElevator;
import frc.robot.commands.ClampShoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static frc.robot.Constants.ManipulatorConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHybridNode extends SequentialCommandGroup {
  /** Creates a new PlaceHybridNode. */
  public PlaceHybridNode(ArmSubsystem arm, ClampSubsystem clamp, ElevatorSubsystem elevator) {
    if(clamp.isClampInCubeMode())
      addCommands(
        new ParallelCommandGroup(
          new BringArm(arm, () -> kArmBumperPosistion, true),
          new BringElevator(elevator, kElevatorLowNodePosition, true)),
        new ClampShoot(clamp).withTimeout(kClampShootDuration),
        new WaitCommand(kPlaceCommandWaitTime),
        new BringArm(arm, () -> kArmInsidePosition, true)
      );
    else
      addCommands(
        new ParallelCommandGroup(
          new BringArm(arm, () -> kArmIntakePosition, true),
          new BringElevator(elevator, kElevatorLowNodePosition, true)),
        new InstantCommand(clamp::toggleSolenoid),
        new WaitCommand(kPlaceCommandWaitTime),
        new BringArm(arm, () -> kArmInsidePosition, true)
      ); 
  }
}
