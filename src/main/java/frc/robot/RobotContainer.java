// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ManipulatorConstants.*;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.BringArmIn;
import frc.robot.commands.BringArmOut;
import frc.robot.commands.BringElevatorUp;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopClamp;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopElevator;
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
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ClampSubsystem m_clampSubsystem = new ClampSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  // Initialize controllers
  XboxController m_driverController = new XboxController(0);
  XboxController m_manipulatorController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_driveSubsystem.setDefaultCommand(new TeleopDrive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightX));
    m_clampSubsystem.setDefaultCommand(new TeleopClamp(m_clampSubsystem, m_manipulatorController::getAButtonPressed, m_manipulatorController::getRightTriggerAxis));
    m_armSubsystem.setDefaultCommand(new TeleopArm(m_armSubsystem, m_manipulatorController::getLeftY));
    m_elevatorSubsystem.setDefaultCommand(new TeleopElevator(m_elevatorSubsystem, m_manipulatorController::getRightY));

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
    new POVButton(m_manipulatorController, 0).onTrue(new BringArmIn(m_armSubsystem).andThen(new BringArmOut(m_armSubsystem).withTimeout(1.5)));
    new POVButton(m_manipulatorController, 90).onTrue(new BringArmIn(m_armSubsystem).andThen(new BringArmOut(m_armSubsystem).withTimeout(1.0)));
    new POVButton(m_manipulatorController, 270).onTrue(new BringArmIn(m_armSubsystem).andThen(new BringArmOut(m_armSubsystem).withTimeout(0.5)));
    new POVButton(m_manipulatorController, 180).onTrue(new BringArmIn(m_armSubsystem));
    new JoystickButton(m_manipulatorController, Button.kRightBumper.value).onTrue(new InstantCommand(m_clampSubsystem::toggleSolenoid));

    new JoystickButton(m_manipulatorController, Button.kLeftBumper.value).onTrue(new BringElevatorUp(m_elevatorSubsystem));
    new JoystickButton(m_manipulatorController, Button.kA.value).onTrue(new PIDCommand(new PIDController(kElevatorMotorP, kElevatorMotorI, kElevatorMotorD), m_elevatorSubsystem::getPosition, kElevatorIntakePosition, m_elevatorSubsystem::setElevatorMotorSpeed, m_elevatorSubsystem));
    new JoystickButton(m_manipulatorController, Button.kB.value).onTrue(new PIDCommand(new PIDController(kElevatorMotorP, kElevatorMotorI, kElevatorMotorD), m_elevatorSubsystem::getPosition, kElevatorLowNodePosition, m_elevatorSubsystem::setElevatorMotorSpeed, m_elevatorSubsystem));
    new JoystickButton(m_manipulatorController, Button.kX.value).onTrue(new PIDCommand(new PIDController(kElevatorMotorP, kElevatorMotorI, kElevatorMotorD), m_elevatorSubsystem::getPosition, kElevatorMidConePosition, m_elevatorSubsystem::setElevatorMotorSpeed, m_elevatorSubsystem));
    new JoystickButton(m_manipulatorController, Button.kY.value).onTrue(new PIDCommand(new PIDController(kElevatorMotorP, kElevatorMotorI, kElevatorMotorD), m_elevatorSubsystem::getPosition, kElevatorHighPosition, m_elevatorSubsystem::setElevatorMotorSpeed, m_elevatorSubsystem));
  }

  // Generates Ramsete command, used for trajectories

  public RamseteCommand getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory, 
      m_driveSubsystem::getPose, 
      new RamseteController(), 
      new SimpleMotorFeedforward(
        ksVolts, 
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter), 
      m_driveSubsystem.getKinematics(), 
      m_driveSubsystem::getWheelSpeeds, 
      new PIDController(kPDriveVel, 0, 0), 
      new PIDController(kPDriveVel, 0, 0), 
      m_driveSubsystem::tankDriveVolts, 
      m_driveSubsystem);
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
    return null;
  }
}
