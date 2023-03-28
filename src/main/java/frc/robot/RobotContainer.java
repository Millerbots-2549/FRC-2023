// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import static frc.robot.Constants.ManipulatorConstants.*;

import java.io.IOException;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.BringArm;
import frc.robot.commands.BringElevator;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.sequences.AutoHighCubeBalance;
import frc.robot.commands.sequences.AutoPlaceCube;
import frc.robot.commands.sequences.BalanceOnChargeStation;
import frc.robot.commands.sequences.DebugDriveSpeedBalancing;
import frc.robot.commands.sequences.Intake;
import frc.robot.commands.sequences.PlaceCubeHigh;
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

  // Chooser for autonomous commands
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getLeftX()), m_driveSubsystem));
    m_clampSubsystem.setDefaultCommand(new RunCommand(() -> m_clampSubsystem.setClampMotorSpeeds(kClampHoldVelocity, kClampHoldCurrentLimit), m_clampSubsystem).handleInterrupt(() -> m_manipulatorController.getAButtonPressed()));
    m_armSubsystem.setDefaultCommand(new BringArm(m_armSubsystem, () -> (m_elevatorSubsystem.getEncoderDistance() > (kElevatorLowNodePosition - kElevatorPositionTolerance)) ? kArmInsidePosition : kArmIntakePosition, false));
    m_elevatorSubsystem.setDefaultCommand(new BringElevator(m_elevatorSubsystem, () -> (m_elevatorSubsystem.getEncoderDistance() > kElevatorLowNodePosition && m_armSubsystem.getEncoderDistance() < kArmBumperPosistion) ? m_elevatorSubsystem.getEncoderDistance() : kElevatorLowNodePosition, false));
    // Configure the button bindings
    configureBindings();

    //Creates options for the command chooser
    m_chooser.addOption("Red/Blue Mid Cube Balance", new AutoHighCubeBalance(m_armSubsystem, m_clampSubsystem, m_elevatorSubsystem, m_driveSubsystem));
    m_chooser.addOption("debug drivetrain", new DebugDriveSpeedBalancing(m_driveSubsystem));
    for(int i = 0; i < 4; i++)
      try{
        Trajectory traj = TrajectoryUtil.fromPathweaverJson(DriveConstants.kAutoTrajectories[i]);
        m_chooser.addOption(DriveConstants.kAutoNames[i], new AutoPlaceCube(m_armSubsystem, m_clampSubsystem, m_elevatorSubsystem, kArmMidConePosition, kElevatorHighPosition).andThen(
          m_driveSubsystem.getRamseteCommand(traj.transformBy(new Transform2d(m_driveSubsystem.getPose(), traj.getInitialPose())))));
      }catch (IOException e){
        e.printStackTrace();
      }
    SmartDashboard.putData(m_chooser);
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
    new JoystickButton(m_driverController, Button.kY.value).onTrue(new DriveStraight(0, false, m_driveSubsystem, m_driveSubsystem.getHeading()).withTimeout(1.0));

    new JoystickButton(m_driverController, Button.kB.value).onTrue(new InstantCommand(() -> m_driveSubsystem.tankDrive(0.0, 0.0), m_driveSubsystem));

    new JoystickButton(m_manipulatorController, Button.kY.value).onTrue(new Intake(m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem, m_manipulatorController));
    new JoystickButton(m_manipulatorController, Button.kB.value).onTrue(new InstantCommand(() -> m_manipulatorController.getAButtonPressed()).andThen(
      new RunCommand(() -> {
        m_clampSubsystem.setClampMotorSpeeds((m_manipulatorController.getLeftTriggerAxis()/2 > kClampIntakeJoystickDeadzone ? m_manipulatorController.getLeftTriggerAxis()/2 * (m_manipulatorController.getRightBumper() ? -1 : 1) : kClampHoldVelocity), (m_manipulatorController.getLeftTriggerAxis()/0.02 > 0.07 ? kClampIntakeCurrentLimit : kClampHoldCurrentLimit));
        if(m_manipulatorController.getAButtonPressed()) m_clampSubsystem.toggleSolenoid();
        m_armSubsystem.setMotorSpeed(m_manipulatorController.getRightY());
        m_elevatorSubsystem.setMotorSpeed(m_manipulatorController.getLeftY());
      }, m_clampSubsystem, m_armSubsystem, m_elevatorSubsystem)));

    new JoystickButton(m_manipulatorController, Button.kX.value).onTrue(new InstantCommand(() -> {
        m_elevatorSubsystem.setMotorSpeed(0.0); 
        m_clampSubsystem.setClampMotorSpeeds(0.0, kClampIntakeCurrentLimit); 
        m_armSubsystem.setMotorSpeed(0.0);
      }, m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem));
    new JoystickButton(m_manipulatorController, Button.kStart.value).onTrue(
      new InstantCommand(() -> m_elevatorSubsystem.resetEncoder()).andThen(
        new InstantCommand(() -> m_armSubsystem.resetEncoder())));

    new POVButton(m_manipulatorController, 180).onTrue(new PlaceHybridNode(m_armSubsystem, m_clampSubsystem, m_elevatorSubsystem, m_manipulatorController::getAButton));
    new POVButton(m_manipulatorController, 90).onTrue(new PlaceMidNode(m_armSubsystem, m_elevatorSubsystem, m_clampSubsystem, m_manipulatorController));
    new POVButton(m_manipulatorController, 0).onTrue(new PlaceCubeHigh(m_armSubsystem, m_clampSubsystem, m_elevatorSubsystem, m_manipulatorController));
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
