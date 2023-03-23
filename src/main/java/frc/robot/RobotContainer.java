// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.time.Instant;

import frc.robot.commands.BringArm;
import frc.robot.commands.BringElevator;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.TeleopClamp;
import frc.robot.commands.sequences.BalanceOnChargeStation;
import frc.robot.commands.sequences.Intake;
import frc.robot.commands.sequences.PlaceHybridNode;
import frc.robot.commands.sequences.PlaceMidNode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClampSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize controllers
  XboxController m_driverController = new XboxController(0);
  XboxController m_manipulatorController = new XboxController(1);
  
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClampSubsystem m_clampSubsystem = new ClampSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getLeftX()), m_driveSubsystem));
    m_clampSubsystem.setDefaultCommand(new TeleopClamp(m_clampSubsystem, m_manipulatorController::getAButtonPressed, m_manipulatorController::getLeftTriggerAxis, m_manipulatorController::getRightBumper));
    m_armSubsystem.setDefaultCommand(new RunCommand(() -> m_armSubsystem.setMotorSpeed(m_manipulatorController.getRightY()), m_armSubsystem));
    m_elevatorSubsystem.setDefaultCommand(new RunCommand(() -> m_elevatorSubsystem.setMotorSpeed(m_manipulatorController.getLeftY()), m_elevatorSubsystem));
    // Configure the button bindings
    configureBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    //new JoystickButton(m_manipulatorController, Button.kA.value).onTrue(new Intake(m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem, m_manipulatorController::getLeftBumperPressed));
    //new JoystickButton(m_manipulatorController, Button.kY.value).onTrue(new GrabFromHumanPlayer(m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem, m_manipulatorController::getBButtonPressed));

    new JoystickButton(m_driverController, Button.kA.value).onTrue(new BalanceOnChargeStation(m_driveSubsystem));
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new DriveStraight(0, false, m_driveSubsystem).withTimeout(1.0));

    new JoystickButton(m_driverController, Button.kB.value).onTrue(new InstantCommand(() -> m_driveSubsystem.tankDrive(0.0, 0.0)));

    new JoystickButton(m_manipulatorController, Button.kY.value).onTrue(new Intake(m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem, m_manipulatorController::getAButtonPressed));
    new JoystickButton(m_manipulatorController, Button.kX.value).onTrue(new PlaceHybridNode(m_armSubsystem, m_clampSubsystem));
    //new JoystickButton(m_manipulatorController, Button.kB.value).onTrue(new PlaceMidNode(m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem));

    new JoystickButton(m_manipulatorController, Button.kB.value).onTrue(new InstantCommand(() -> {m_elevatorSubsystem.setMotorSpeed(0.0); m_clampSubsystem.setClampMotorSpeeds(0.0, 0.0); m_armSubsystem.setMotorSpeed(0.0);}, m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem));
    new JoystickButton(m_manipulatorController, Button.kLeftBumper.value).onTrue(new InstantCommand(() -> m_elevatorSubsystem.resetEncoder()).andThen(new InstantCommand(() -> m_armSubsystem.resetEncoder())));

    new POVButton(m_manipulatorController, 0).onTrue(new BringElevator(m_elevatorSubsystem, 0.0));
    new POVButton(m_manipulatorController, 90).onTrue(new BringElevator(m_elevatorSubsystem, 0.40));
    new POVButton(m_manipulatorController, 180).onTrue(new BringElevator(m_elevatorSubsystem, 0.80));
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Runs this command in autonomous
    //m_driveSubsystem.resetOdometry(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    //return getRamseteCommand(TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 1, new Rotation2d(Math.PI/2)), m_driveSubsystem.getTrajectoryConfig())).andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
    return new BringArm(m_armSubsystem, kArmBumperPosistion);
  }
}
