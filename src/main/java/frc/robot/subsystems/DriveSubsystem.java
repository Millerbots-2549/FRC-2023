// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TrajectoryConstants;

public class DriveSubsystem extends SubsystemBase {

  //Motors on left side of drive
  private final MotorControllerGroup m_leftMotors = 
    new MotorControllerGroup(
      new WPI_TalonSRX(DriveConstants.kLeftMotor1Port), 
      new WPI_TalonSRX(DriveConstants.kLeftMotor2Port));

  //Motors on right side of drive
  private final MotorControllerGroup m_rightMotors = 
    new MotorControllerGroup(
      new WPI_TalonSRX(DriveConstants.kRightMotor1Port), 
      new WPI_TalonSRX(DriveConstants.kRightMotor2Port));

  //Robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Left/right side drive encoders
  private final Encoder m_leftEncoder =
    new Encoder(
      DriveConstants.kLeftEncoderPorts[0], 
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);
  private final Encoder m_rightEncoder =
    new Encoder(
      DriveConstants.kLeftEncoderPorts[0], 
      DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);

  // Gyrometer sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Object that contains all odometric data (distance, angle, etc. of robot)
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_rightMotors.setInverted(true);

    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  @Override
  public void periodic() {
    // Update odometry constantly
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters(); //returns estimated pose (position) of robot
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(null, m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  // Drives the drivetrain using two control schemes: arcade and tank

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot); 
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  // Controls the drivetrain directly with voltage
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  // Zeroes out the encoders of the robot
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // Get the average distance of the two encoders
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  // Gets the left/right encoders

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  // Sets max output of drive, used for scaling the drive to drive slowly
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  // Zeroes the heading of the robot
  public void zeroHeading() {
    m_gyro.reset();
  }

  // Returns heading of robot, with range -180 to 180
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  // Returns turn rate of robot, in degrees/sec
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  // Creates voltage constraints to limit acceleration during trajectories
  public DifferentialDriveVoltageConstraint getVoltageConstraint() {
    return new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter), 
        DriveConstants.kDriveKinematics, 
        10);
  }

  // Creates config for robot trajectories
  public TrajectoryConfig getTrajectoryConfig() {
    return new TrajectoryConfig(
      TrajectoryConstants.kMaxSpeedMetersPerSecond, 
      TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics)
      .addConstraint(getVoltageConstraint());
  }
}
