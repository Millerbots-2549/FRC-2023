// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants { //TODO: add all of these
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;

        public static final int[] kLeftEncoderPorts = {2, 3};
        public static final int[] kRightEncoderPorts = {0, 1};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;

        public static final double kEncoderDistancePerPulse = (Units.inchesToMeters(6.0)*Math.PI)/360;

        public static final double kDriveAutoSpeed = 0.4;

        public static final double kPDriveVel = 5.0952;
        public static final double ksVolts = 1.0493;
        public static final double kvVoltSecondsPerMeter = 3.2019;
        public static final double kaVoltSecondsSquaredPerMeter = 0.96527;

        public static final double kMaxSpeedMetersPerSecond = 0.75; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kBalanceAirPitch = -14.0; // TODO: test
        public static final double kBalanceDropPitch = -10.0;
        public static final double kBalanceWheelSpeedHigh = -0.80;
        public static final double kBalanceWheelSpeedLowVolts = -3.5;

        public static final double kTrackWidthMeters = 0.56394;
        public static double kMaxJoystickAcceleration;

        public static final Path[] kAutoTrajectories = {
            Filesystem.getDeployDirectory().toPath().resolve("paths/AvoidBlueLeft.wpilib.json"),
            Filesystem.getDeployDirectory().toPath().resolve("paths/AvoidBlueRight.wpilib.json"),
            Filesystem.getDeployDirectory().toPath().resolve("paths/AvoidRedLeft.wpilib.json"),
            Filesystem.getDeployDirectory().toPath().resolve("paths/AvoidRedRight.wpilib.json")
        };
        public static final String[] kAutoNames = {
            "Blue Left Cube",
            "Blue Right Cube",
            "Red Left Cube",
            "Red Right Cube"
        };
    }

    public static final class ManipulatorConstants {
        public static final int kArmMotorPort = 7;
        public static final int kLeftClampMotorPort = 6;
        public static final int kRightClampMotorPort = 5;
        public static final int kElevatorMotorPort = 8;
        public static final int kClampSolenoidPort = 1;

        public static final int[] kArmEncoderPorts = {4, 5};
        public static final double kArmEncoderDistancePerPulseMeters = 0.025 * Math.PI / 2048;

        public static final double kArmJoystickDeadzone = 0.05;

        public static final double kArmCalibrationSpeed = 0.75;
        public static final double kArmMotorStallCurrent = 19.5;

        public static final double kArmInsidePosition = 0.0;
        public static final double kArmBumperPosistion = -0.05;
        public static final double kArmIntakePosition = -0.35;
        public static final double kArmMidCubePosition = -0.30;
        public static final double kArmMidConePosition = -0.70;

        public static final double kArmFeedforwardKS = 1.2541;
        public static final double kArmFeedforwardKG = 1.5599;
        public static final double kArmFeedforwardKV = 10.2778;
        public static final double kArmFeedforwardKA = 1.1208;
        public static final double kArmMotorKP = 25.9;
        public static final double kArmMaxVelocity = 0.7;
        public static final double kArmMaxAcceleration = 1.4;
        public static final double kArmPositionTolerance = 0.03; 

        public static final double kClampIntakeJoystickDeadzone = 0.07;

        public static final double kClampIntakeVelocity = 0.5;
        public static final int kClampIntakeCurrentLimit = 20;
        public static final double kClampHoldVelocity = 0.07;
        public static final int kClampHoldCurrentLimit = 5;
        public static final double kClampShootVelocity = -0.75;
        public static final double kClampShootDuration = 0.5;
        public static final double kClampMotorStartTime = 0.5;

        public static final double kClampVelocityDeadzone = 1.0;

        public static final double kElevatorSprocketDiameterMeters = 0.0363728;
        public static final double kElevatorDistancePerPulse = Units.inchesToMeters(1.432*Math.PI/36);

        public static final double kElevatorJoystickDeadzone = 0.05;

        public static final double kElevatorMotorStallCurrent = 0.0; // TODO: add
        public static final double kElevatorMotorAutoSpeed = 0.75;

        public static final double kElevatorIntakePosition = -0.20; 
        public static final double kElevatorLowNodePosition = 0.0;
        public static final double kElevatorMidCubePosistion = 0.5;
        public static final double kElevatorHighPosition = 0.83;
        public static final double kElevatorHighestPosition = 0.85;

        public static final double kElevatorFeedforwardKS = 0.1389;
        public static final double kElevatorFeedforwardKG = 0.14061;
        public static final double kElevatorFeedforwardKV = 39.188;
        public static final double kElevatorFeedforwardKA = 0.58921;
        public static final double kElevatorMotorKP = 12.0;
        public static final double kElevatorMaxVelocity = 0.25;
        public static final double kElevatorMaxAcceleration = 1.0;
        public static final double kElevatorPositionTolerance = 0.01;

        public static final double kPlaceCommandWaitTime = 0.1;
    }

    public static final class VisionConstants {
        public static final String kCameraName = "OV5647";
        public static final Transform3d kRobotToCamTransform = new Transform3d(new Translation3d(0, 0, 1.3255625), new Rotation3d(0, Units.degreesToRadians(15), 0)); // TODO: add

        // Defining objects representing AprilTags on the field
        public static final Translation3d[] AprilTagLocations = {
                new Translation3d(15.513558, 1.071626, 0.462788),
                new Translation3d(15.513558, 2.748026, 0.462788),
                new Translation3d(15.513558, 4.424426, 0.462788),
                new Translation3d(16.178784, 6.749796, 0.695452),
                new Translation3d(0.36195, 6.749796, 0.695452),
                new Translation3d(1.02743, 4.424426, 0.462788),
                new Translation3d(1.02743, 2.748026, 0.462788),
                new Translation3d(1.02743, 1.071626, 0.462788)
        };
        public static final Rotation3d[] AprilTagOrientations = {
            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)),
            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)),
            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)),
            new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0)),
            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)),
            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)),
            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)),
            new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)),
        };
        public static final AprilTagFieldLayout kFieldLayout = 
            new AprilTagFieldLayout(
                new ArrayList<AprilTag>() {
                    {
                        for(int i = 0; i < 8; i++){
                            add(new AprilTag(i+1, new Pose3d(AprilTagLocations[i], AprilTagOrientations[i])));
                        }
                    }
                },
                16.54175,
                8.0137);
    }

}
