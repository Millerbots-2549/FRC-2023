// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.BringArm;
import frc.robot.commands.BringElevator;
import frc.robot.commands.ClampShoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.util.function.BooleanSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceMidNode extends SequentialCommandGroup {
  /** Creates a new PlaceMidNode. */
  public PlaceMidNode(ArmSubsystem arm, ElevatorSubsystem elevator, ClampSubsystem clamp, BooleanSupplier wait) {
    SmartDashboard.putBoolean("clamp mode command supplier", clamp.isClampInCubeMode());
    if(clamp.isClampInCubeMode()){
      SmartDashboard.putBoolean("command clamp mode", true);
      addCommands(
        new ParallelCommandGroup(
          new BringElevator(elevator, kElevatorMidCubePosistion, true),
          new BringArm(arm, () -> kArmMidCubePosition, true)
        ),
        new WaitUntilCommand(wait),
        new ClampShoot(clamp).withTimeout(kClampShootDuration),
        new WaitCommand(kPlaceCommandWaitTime),
        new ParallelCommandGroup(
          new BringElevator(elevator, kElevatorLowNodePosition, true),
          new BringArm(arm, () -> kArmInsidePosition, true)
        )
      );
    }else{
      SmartDashboard.putBoolean("command clamp mode", false);
      addCommands(
        new BringElevator(elevator, kElevatorHighPosition, true),
        new BringArm(arm, () -> kArmMidConePosition, true),
        new WaitUntilCommand(wait),
        new InstantCommand(clamp::toggleSolenoid, clamp),
        new WaitCommand(kPlaceCommandWaitTime),
        new BringArm(arm, () -> kArmInsidePosition, true),
        new BringElevator(elevator, kElevatorLowNodePosition, true)
      );
      addRequirements(arm, elevator, clamp);
    }
  }
}
