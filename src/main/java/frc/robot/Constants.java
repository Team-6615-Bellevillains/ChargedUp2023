package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final int kCimcoderPulsesPerRevolution = 20;
    public static final int kCimcoder256PulsesPerRevolution = 256;

    public static final class SwerveModuleConstants {
        public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;
        public static final double kDriveMotorGearRatio = 6.67 / 1;
        public static final double kSteerMotorGearRatio = 48.0 / 40.0;
        public static final double kPTurning = 1.75;
        public static final double kITurning = 0;
        public static final double kDTurning = 0.0;

        public static final double kDriveEncoderRot2Meter = kWheelCircumference / kDriveMotorGearRatio;
        public static final double kSteerEncoderRot2Rad = 2 * Math.PI / kSteerMotorGearRatio;

        public static final int maximumTotalCounts = 1024;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.662; // Distance between right and left wheels
        public static final double kWheelBase = 0.504; // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 9;
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

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetCounts = 135;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetCounts = 857;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetCounts = 282;
        public static final double kBackRightDriveAbsoluteEncoderOffsetCounts = 777;

        /*
         * wheelPoint = (0,0) or (0,kWheelBase) or (kTrackWidth, 0) or (kTrackWidth,
         * kWheelBase)
         * centerPoint = (kTrackWidth/2, kWheelBase/2)
         * a = abs(centerPoint_x-wheelPoint_x) -> evaluates to kTrackWidth/2
         * b = abs(centerPoint_y-wheelPoint_y) -> evaluates to kWheelBase/2
         * kWheelDistanceFromCenter^2 = a^2 + b^2
         * kWheelDistanceFromCenter = sqrt(a^2+b^2)
         */
        public static final double kWheelDistanceFromCenter = Math.sqrt(Math.pow(kTrackWidth / 2, 2) + Math.pow(kWheelBase / 2, 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(13.5);

        /*
         * 2pi radians / (Circumference of the circle created by robot rotation aka the distance travelled in one rotation / max speed)
         */
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI / (kWheelDistanceFromCenter / kPhysicalMaxSpeedMetersPerSecond);

        public static final double kTeleOpMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleOpMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 35;
        public static final double kTeleOpMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleOpMaxAngularAccelerationUnitsPerSecond = 3;

        // TODO: Tune
        public static final double kPRotation = .25;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0;

        // TODO: Tune
        public static final double kPMoveXDistance = 0;
        public static final double kIMoveXDistance = 0;
        public static final double kDMoveXDistance = 0;

        // TODO: Tune
        public static final double kPMoveYDistance = 0;
        public static final double kIMoveYDistance = 0;
        public static final double kDMoveYDistance = 0;

    }

    public static final class ElevatorConstants {
        /*
         * We are unsure about the integer values needed here right now. Temp values are
         * in place
         */

        // Begin Vertical
        public static final int verticalMotorAPort = 17;
        public static final int verticalMotorBPort = 3;

        public static final double verticalGearRatio = 12.0 / 60.0;
        public static final double verticalMaxHeight = Units.inchesToMeters(30);
        public static final double verticalGearDiameter = Units.inchesToMeters(1.76);

        public static final int verticalEncoderPulsesPerRevolution = kCimcoderPulsesPerRevolution;
        public static final double verticalRotationsToDistance = verticalGearDiameter * Math.PI * verticalGearRatio;

        public static final double verticalLowHeight = 0;
        public static final double verticalMidHeight = Units.inchesToMeters(15);
        public static final double verticalHighHeight = verticalMaxHeight - Units.inchesToMeters(1);

        public static final double kPVerticalElevator = 0;
        public static final double kIVerticalElevator = 0;
        public static final double kDVerticalElevator = 0;

        public static final double kSVerticalElevator = 3.750000;
        public static final double kGVerticalElevator = 0.032000;
        public static final double kVVerticalElevator = 2;
        public static final double kAVerticalElevator = 0;

        // Begin Horizontal
        public static final int horizontalMotorPort = 18;

        public static final double horizontalGearRatio = 12.0 / 60.0;
        public static final double horizontalMaxExtensionLength = Units.inchesToMeters(15);
        public static final double horizontalGearDiameter = Units.inchesToMeters(1.76);

        public static final int horizontalEncoderPulsesPerRevolution = kCimcoder256PulsesPerRevolution;
        public static final double horizontalRotationsToDistance = horizontalGearDiameter * Math.PI * horizontalGearRatio;

        public static final double horizontalInLength = 0;
        public static final double horizontalOutLength = horizontalMaxExtensionLength-Units.inchesToMeters(1);

        public static final double kPHorizontalElevator = 0;
        public static final double kIHorizontalElevator = 0;
        public static final double kDHorizontalElevator = 0;

        public static final double kSHorizontalElevator = 0;
        public static final double kVHorizontalElevator = 0;
        public static final double kAHorizontalElevator = 0;
    }

    public static final class GrabberConstants {
        public static final int kLeftRollerMotorPort = 15;
        public static final int kRightRollerMotorPort = 14;

        public static final int kLeftSolenoidForwardChannel = 1;
        public static final int kLeftSolenoidReverseChannel = 2;
        public static final int kRightSolenoidForwardChannel = 3;
        public static final int kRightSolenoidReverseChannel = 4;

        public static final int kFlipMotorPort = 16;
        public static final double kFlipReverseThreshold = 0;

        /*
         * Dr. Blake "MillieB" Miller Expert Calculation
         * (PhD in Mechanical Engineering)
         *
         * Motor drives 12 tooth gear that drives a 60 tooth gear that is coaxial with a
         * 16 tooth gear that drives a 58 tooth gear that is coaxial with a 12 tooth
         * gear that drives a 30 tooth gear via chain. :')
         */
        public static final double kFlipGearRatio = (12.0 / 60.0) * (16.0 / 58.0) * (12.0 / 30.0);

        public static final int flipPulsesPerRevolution = kCimcoder256PulsesPerRevolution;
        public static final double flipRotationsToRadians = 2 * Math.PI / kFlipGearRatio;

        public static final double kPFlip = 0.1;
        public static final double kIFlip = 0.1;
        public static final double kDFlip = 0.1;

        public static final double grabberInSetpoint = 0;
        public static final double grabberShootSetpoint = Units.degreesToRadians(118 - 25);
        public static final double grabberIntakeSetpoint = Units.degreesToRadians(118 + 27);
    }

    public static final class LimelightConstants {
        public static final String kCameraName = "asdoija";
    }

    public static final class AutoConstants {
        public static final double kAutoMaxSpeedMetersPerSecond = 1;
        public static final double kAutoMaxAccelerationMetersPerSecond = 0.5;

        public static final double kPTrackingYaw = .05;
        public static final double kPTrackingDrive = .7;
        public static final double kTrackingDistance = .45;
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.07;
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

}