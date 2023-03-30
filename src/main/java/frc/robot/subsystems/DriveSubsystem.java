// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.Constants.VisionConstants;
import frc.robot.VisionContainer;

public class DriveSubsystem extends SubsystemBase {

  //Motors on left side of drive
  private final MotorControllerGroup m_leftMotors = 
    new MotorControllerGroup(
      new WPI_TalonSRX(kLeftMotor1Port), 
      new WPI_TalonSRX(kLeftMotor2Port));

  //Motors on right side of drive
  private final MotorControllerGroup m_rightMotors = 
    new MotorControllerGroup(
      new WPI_TalonSRX(kRightMotor1Port), 
      new WPI_TalonSRX(kRightMotor2Port));

  //Robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Left/right side drive encoders
  private final Encoder m_leftEncoder =
    new Encoder(
      kLeftEncoderPorts[0], 
      kLeftEncoderPorts[1],
      kLeftEncoderReversed);
  private final Encoder m_rightEncoder =
    new Encoder(
      kRightEncoderPorts[0], 
      kRightEncoderPorts[1],
      kRightEncoderReversed);

  // Gyrometer sensor
  private final ADIS16448_IMU m_gyro = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._32s); 
  
  // Object that converts between robot speeds and wheel speeds
  private final DifferentialDriveKinematics m_kinematics = 
    new DifferentialDriveKinematics(kTrackWidthMeters);

  // Object that estimates robot's pose with MATH (look it up if you're curious)
  private final DifferentialDrivePoseEstimator m_poseEstimator = 
    new DifferentialDrivePoseEstimator(
      m_kinematics,
      new Rotation2d(Units.degreesToRadians(getHeading())), 
      m_leftEncoder.getDistance(), 
      m_rightEncoder.getDistance(), 
      new Pose2d());

  // Vision system class
  private VisionContainer visionContainer;

  // TODO: add SlewRateLimiter somewhere to limit drive acceleration

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_rightMotors.setInverted(true);

    m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
  
    resetEncoders();

    visionContainer = new VisionContainer();
  }

  @Override
  public void periodic() {
    // Update odometry constantly
    updateOdometry();
    SmartDashboard.putNumber("x gyro", m_gyro.getGyroAngleX());
    SmartDashboard.putNumber("y gyro", m_gyro.getGyroAngleY());
    SmartDashboard.putNumber("z gyro", m_gyro.getGyroAngleZ());
    SmartDashboard.putNumber("closest node x", getClosestNode().getX());
    SmartDashboard.putNumber("closest node y", getClosestNode().getY());
    SmartDashboard.putNumber("left wheel speed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("right wheel speed", getWheelSpeeds().rightMetersPerSecond);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition(); //returns estimated pose (position) of robot
  }

  public Translation2d getClosestNode() {
    Translation2d closestTag = new Translation2d();
    Translation2d closestNode = new Translation2d();
    double closestDistanceMeters = 30;
    for(int i = 0; i < 8; i++){
      if(getPose().getTranslation().getDistance(VisionConstants.AprilTagLocations[i].toTranslation2d()) < closestDistanceMeters){
        closestTag = VisionConstants.AprilTagLocations[i].toTranslation2d();
        closestDistanceMeters = getPose().getTranslation().getDistance(VisionConstants.AprilTagLocations[i].toTranslation2d());
      }
    }
    double xTag = closestTag.getX();
    double yTag = closestTag.getY();
    double xOffset = (closestTag.getX() < 7) ? 0.36195 : -0.36195;
    double yOffset = 0.32305625;
    Translation2d[] possibleNodes = {
      new Translation2d(xTag+xOffset, yTag-yOffset),
      new Translation2d(xTag+xOffset, yTag),
      new Translation2d(xTag+xOffset, yTag+yOffset),
    };
    for(int i = 0; i < 3; i++){
      if(getPose().getTranslation().getDistance(possibleNodes[i]) < closestDistanceMeters){
        closestNode = possibleNodes[i];
        closestDistanceMeters = getPose().getTranslation().getDistance(possibleNodes[i]);
      }
    }

    // TODO: add rest of this shiz
    return closestNode;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void updateOdometry() {
    m_poseEstimator.update(new Rotation2d(Units.degreesToRadians(getHeading())), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    Optional<EstimatedRobotPose> result = visionContainer.getEstimatedRobotPose();

    if(result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_poseEstimator.addVisionMeasurement(
        camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      SmartDashboard.putNumber("Vision Est. X Pos", camPose.estimatedPose.getX());
      SmartDashboard.putNumber("Vision Est. Y Pos", camPose.estimatedPose.getY());
      SmartDashboard.putNumber("Vision Est. Z Pos", camPose.estimatedPose.getZ());
      SmartDashboard.putNumber("Vision Est. Theta", camPose.estimatedPose.toPose2d().getRotation().getDegrees());
      SmartDashboard.putNumber("Vision Timestamp", camPose.timestampSeconds);
    }

    SmartDashboard.putNumber("Pose X", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Pose Y", m_poseEstimator.getEstimatedPosition().getY());
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
    SmartDashboard.putNumber("left volts", leftVolts);
    SmartDashboard.putNumber("right volts", rightVolts);
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

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_poseEstimator.resetPosition(new Rotation2d(Units.degreesToRadians(getHeading())), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);  
  }

  // Returns heading of robot, with range -180 to 180
  public double getHeading() {
    return -m_gyro.getGyroAngleZ();
  }

  // Returns turn rate of robot, in degrees/sec
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  // Creates voltage constraints to limit acceleration during trajectories
  public DifferentialDriveVoltageConstraint getVoltageConstraint() {
    return new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        ksVolts, 
        kvVoltSecondsPerMeter, 
        kaVoltSecondsSquaredPerMeter), 
        m_kinematics, 
        10);
  }

  // Creates config for robot trajectories
  public TrajectoryConfig getTrajectoryConfig(double maxSpeed) {
    if(maxSpeed != 0)
      return new TrajectoryConfig(
        maxSpeed, 
        maxSpeed*2)
        .setKinematics(m_kinematics)
        .addConstraint(getVoltageConstraint());
    else
      return new TrajectoryConfig(
        kMaxSpeedMetersPerSecond, 
        kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(m_kinematics)
        .addConstraint(getVoltageConstraint());
  }
  public DifferentialDriveKinematics getKinematics() {
    return m_kinematics;
  }

  // Generates Ramsete command, used for trajectories

  public RamseteCommand getRamseteCommand(Trajectory trajectory) {
    return new RamseteCommand(
      trajectory, 
      () -> getPose(), 
      new RamseteController(), 
      new SimpleMotorFeedforward(
        ksVolts, 
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter), 
      m_kinematics, 
      () -> getWheelSpeeds(), 
      new PIDController(kPDriveVel, 0, 0), 
      new PIDController(kPDriveVel, 0, 0), 
      (a, b) -> tankDriveVolts(a, b),
      this);
  }

  double[] a = {1, 5};

  public double getRobotPitch() {
    return m_gyro.getGyroAngleY();
  }

  public void resetGyro() {
    m_gyro.reset();
  }
}
