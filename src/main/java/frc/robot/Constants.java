package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class SwerveModuleConstants {
        public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;
        public static final double kDriveMotorGearRatio = 6.67 / 1;
        public static final double kSteerMotorGearRatio = 48 / 40;
        public static final double kPTurning = 1.7;
        public static final double kITurning = 0;
        public static final double kDTurning = 0;

        public static final double kDriveEncoderRot2Meter = kWheelCircumference / kDriveMotorGearRatio;
        public static final double kSteerEncoderRot2Rad = 2 * Math.PI / kSteerMotorGearRatio;

        public static final double maximumTotalCounts = 1024;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 12;
        /*public static final int kBackRightDriveMotorPort = 10;*/

        public static final int kFrontLeftSteerMotorPort = 4;
        public static final int kFrontRightSteerMotorPort = 6;
        public static final int kBackLeftSteerMotorPort = 1;
        public static final int kBackRightSteerMotorPort = 5;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetCounts = 138;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetCounts = 349;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetCounts = 280;
        public static final double kBackRightDriveAbsoluteEncoderOffsetCounts = 243;

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(12);
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 90 / 60 * 2 * Math.PI;

        public static final double kTeleOpMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleOpMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 4;
        public static final double kTeleOpMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleOpMaxAngularAccelerationUnitsPerSecond = 3;

    }

    public static final class OIConstants {
        public static final double kDeadband = 0.15;
        public static final int xboxControllerPort = 0;
        public static final int xboxControllerPort2 = 3;
    }

    public static final class ArmConstants {
        public static final double armMotor = 10;
        public static final double armEncoderPort1 = 0;
        public static final double armEncoderPort2 = 1;
        public static final double armEncoderPPR = 40;
        public static final double armEncoderRateInDegrees = 360 / armEncoderPPR;
        public static final double armEncoderTopValue = 360;

    }   


}
