package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class SwerveModuleConstants {
        public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;
        public static final double kDriveMotorGearRatio = 6.67 / 1;
        public static final double kSteerMotorGearRatio = 48 / 40;
        public static final double kPTurning = 1.75;
        public static final double kITurning = 0;
        public static final double kDTurning = 0;

        public static final double kDriveEncoderRot2Meter = kWheelCircumference / kDriveMotorGearRatio;
        public static final double kSteerEncoderRot2Rad = 2 * Math.PI / kSteerMotorGearRatio;

        public static final double maximumTotalCounts = 1024;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(24); // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20); // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 12;
        public static final int kBackRightDriveMotorPort = 10;

        public static final int kFrontLeftSteerMotorPort = 4;
        public static final int kFrontRightSteerMotorPort = 6;
        public static final int kBackLeftSteerMotorPort = 1;
        public static final int kBackRightSteerMotorPort = 5;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetCounts = 137;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetCounts = 857;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetCounts = 275;
        public static final double kBackRightDriveAbsoluteEncoderOffsetCounts = 777;

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(12);
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 90 / 60 * 2 * Math.PI;

        public static final double kTeleOpMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleOpMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 4;
        public static final double kTeleOpMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleOpMaxAngularAccelerationUnitsPerSecond = 3;

        // TODO: Tune
        public static final double kPRotation = .25;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0;

    }

    public static final class GrabberConstants {
        public static final int kLeftRollerMotorPort = 15;
        public static final int kRightRollerMotorPort = 16;

        public static final int kLeftSolenoidForwardChannel = 1;
        public static final int kLeftSolenoidReverseChannel = 2;
        public static final int kRightSolenoidForwardChannel = 3;
        public static final int kRightSolenoidReverseChannel = 4;

        public static final double kPFlip = 0.1;
        public static final double kIFlip = 0.1;
        public static final double kDFlip = 0.1;
    }


    public static final class AutoConstants {
        public static final double kAutoMaxSpeedMetersPerSecond = 0.5;

        public static final double kPTrackingYaw = .05;
        public static final double kPTrackingDrive = .7;
        public static final double kTrackingDistance = .45;
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.06;
        public static final int kDriverControllerPort = 1;
        public static final int kOperatorControllerPort = 3;

        public static final int kLeftXAxis = 0;
        public static final int kLeftYAxis = 1;
        public static final int kRightXAxis = 4;

        public static final int leftBumper = 5;
        public static final int rightBumper = 6;
        public static final int triangle = 4;
        public static final int shareButton = 7;
        public static final int optionsButton = 8;
    }

}
