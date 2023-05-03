// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BringArm;
import frc.robot.commands.BringElevator;
import frc.robot.commands.ClampShoot;
import frc.robot.commands.ZeroArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceCubeHigh extends SequentialCommandGroup {
  /** Creates a new AutoPlaceCube. */
  public AutoPlaceCubeHigh(ArmSubsystem arm, ClampSubsystem clamp, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> arm.setMotorSpeed(0.75), arm).withTimeout(0.5),
      new ParallelRaceGroup(
        new BringElevator(elevator, () -> kElevatorHighPosition, true),
        new ZeroArm(arm).andThen(new BringArm(arm, () -> kArmInsidePosition, false))
      ),
      new BringArm(arm, () -> kArmMidConePosition, true),
      new ClampShoot(clamp).withTimeout(kClampShootDuration),
      new BringArm(arm, () -> kArmInsidePosition, true),
      new BringElevator(elevator, () -> kElevatorMidCubePosistion, true).raceWith(new BringArm(arm, () -> kArmInsidePosition, false))
    );
  }
}
