// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BringArm;
import frc.robot.commands.BringElevator;
import frc.robot.commands.ClampIntake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabFromHumanPlayer extends SequentialCommandGroup {
  /** Creates a new GrabFromHumanPlayer. */
  public GrabFromHumanPlayer(ArmSubsystem arm, ElevatorSubsystem elevator, ClampSubsystem clamp, XboxController controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BringElevator(elevator, () -> kElevatorHighPosition, true).raceWith(
        new BringArm(arm, () -> kArmInsidePosition, false)),
      new ClampIntake(clamp, controller).withTimeout(0.5),
      new ParallelCommandGroup(
        new ClampIntake(clamp, controller),
        new RunCommand(() -> arm.setMotorSpeed(controller.getRightY()))
      ).until(() -> clamp.getAverageMotorSpeeds() < kClampVelocityDeadzone),
      new SequentialCommandGroup(
        new BringElevator(elevator, () -> kElevatorHighestPosition, true),
        new ParallelRaceGroup(
          new BringArm(arm, () -> kArmInsidePosition, false),
          new WaitUntilCommand(() -> arm.getEncoderDistance() > kArmBumperPosistion).andThen(new BringElevator(elevator, () -> kElevatorMidCubePosistion, true))
        )
      ).raceWith(new RunCommand(() -> clamp.setClampMotorSpeeds(kClampHoldVelocity, kClampHoldCurrentLimit), clamp))
    );
  }
}
