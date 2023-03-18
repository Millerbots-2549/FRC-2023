// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

        public static final int[] kLeftEncoderPorts = {0, 1};
        public static final int[] kRightEncoderPorts = {2, 3};
        public static final boolean kLeftEncoderReversed = true;
        public static final boolean kRightEncoderReversed = false;

        public static final double kEncoderDistancePerPulse = (Units.inchesToMeters(6.0)*Math.PI)/360;

        public static final double kPDriveVel = 0.093432;
        public static final double ksVolts = 0.54397;
        public static final double kvVoltSecondsPerMeter = 3.203818;
        public static final double kaVoltSecondsSquaredPerMeter = 0.7168503;

        public static final double kMaxSpeedMetersPerSecond = 4; 
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;

        public static final double kTrackWidthMeters = Units.inchesToMeters(21.5);
        public static double kMaxJoystickAcceleration;
    }

    public static final class ManipulatorConstants {
        public static final int kArmMotorPort = 7;
        public static final int kLeftClampMotorPort = 6;
        public static final int kRightClampMotorPort = 5;
        public static final int kElevatorMotorPort = 8;
        public static final int kClampSolenoidPort = 0;

        public static final int[] kArmEncoderPorts = {4, 5};
        public static final double kArmEncoderDistancePerPulseMeters = 0.025 * Math.PI / 2048;

        public static final double kArmMotorStallCurrent = 16.0;
        public static final double kArmMotorAutoSpeed = 0.5;
        public static final double kArmMotorGrabSpeed = 0.2;

        public static final double kArmInsidePosition = 0.0;
        public static final double kArmBumperPosistion = -0.15;
        public static final double kArmIntakePosition = 0.0;
        public static final double kArmMidCubePosition = 0.0;
        public static final double kArmMidConePosition = 0.0;

        public static final double kArmFeedforwardKS = 1.8541;
        public static final double kArmFeedforwardKG = 1.5599;
        public static final double kArmFeedforwardKV = 3.2778;
        public static final double kArmFeedforwardKA = 0.73104;
        public static final double kArmMotorKP = 1.0;
        public static final double kArmMaxVelocity = 0.2;
        public static final double kArmMaxAcceleration = 0.5;
        public static final double kArmPositionTolerance = 0.01; 

        public static final double kClampIntakeVelocity = -0.5;
        public static final double kClampShootVelocity = 0.75;
        public static final double kClampShootDuration = 0.5;

        public static final double kClampVelocityDeadzone = 1.0;

        public static final double kElevatorSprocketDiameterMeters = 0.0363728;

        public static final double kElevatorMotorStallCurrent = 0.0; // TODO: add
        public static final double kElevatorMotorAutoSpeed = 0.75;

        public static final double kElevatorIntakePosition = 0.0; 
        public static final double kElevatorLowNodePosition = 0.0;
        public static final double kElevatorMidCubePosistion = 0.0;
        public static final double kElevatorMidConePosition = 0.0;
        public static final double kElevatorGrabPosition = 0.0;
        public static final double kElevatorHighPosition = 0.0;

        public static final double kElevatorFeedforwardKS = 0.0;
        public static final double kElevatorFeedforwardKG = 0.0;
        public static final double kElevatorFeedforwardKV = 0.0;
        public static final double kElevatorFeedforwardKA = 0.0;
        public static final double kElevatorMotorKP = 0.0;
        public static final double kElevatorMaxVelocity = 0.0;
        public static final double kElevatorMaxAcceleration = 0.0;
        public static final double kElevatorPositionTolerance = 0.0;

        public static final double kPlaceCommandWaitTime = 2.0;
    }

    public static final class VisionConstants {
        public static final String kCameraName = "OV5647";
        public static final Transform3d kRobotToCamTransform = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)); // TODO: add

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
