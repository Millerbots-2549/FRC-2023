// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BringArmPID;
import frc.robot.commands.BringElevatorPID;
import frc.robot.commands.ClampIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabFromHumanPlayer extends SequentialCommandGroup {
  /** Creates a new GrabFromHumanPlayer. */
  public GrabFromHumanPlayer(ArmSubsystem arm, ElevatorSubsystem elevator, ClampSubsystem clamp, BooleanSupplier grabCondition) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new BringElevatorPID(elevator, kElevatorGrabPosition),
        new BringArmPID(arm, kArmBumperPosistion)
      ),
      new WaitUntilCommand(grabCondition),
      new ParallelCommandGroup(
        new ClampIntake(clamp, () -> false),
        new RunCommand(() -> arm.setArmMotorSpeed(kArmMotorGrabSpeed))
      ).until(() -> clamp.getAverageMotorSpeeds() < kClampVelocityDeadzone),
      new BringElevatorPID(elevator, kElevatorHighPosition),
      new BringArmPID(arm, kArmInsidePosition),
      new BringElevatorPID(elevator, kElevatorLowNodePosition)
    );
  }
}
