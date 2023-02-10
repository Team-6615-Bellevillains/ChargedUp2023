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
        public static final int kFrontRightDriveMotorPort =9;
        public static final int kBackLeftDriveMotorPort = 12;
        public static final int kBackRightDriveMotorPort = 10;

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

    public static final class ElevatorConstants {
        /* We are unsure about the integer values needed here right now.*/
        
        //vertical elevator
        public static final int vElevatorMotor1 = ;
        public static final int vElevatorMotor2 = ;
        public static final int vElevatorEncoderPort1 = ;
        public static final int vElevatorEncoderPort2 = ;
        public static final int vElevatorGearDiameter = ;
        public static final int vElevatorEncoderPPR = 5; 
        public static final double vElevatorEncoderDPR = ElevatorConstants.vElevatorGearDiameter * Math.PI/ElevatorConstants.vElevatorEncoderPPR;
        public static final int vElevatorEncoderTopValue = ;
        public static final double vElevatorEncoderBottomValue = 0;
        

        //horizontal elevator
        public static final int hElevatorMotor = ;
        public static final int hElevatorEncoderPort1 = ;
        public static final int hElevatorEncoderPort2 = ;
        public static final int hElevatorGearDiameter = ;
        public static final int hElevatorEncoderPPR = 5; //encoder pulses per rotation
        public static final double hElevatorEncoderDPR = ElevatorConstants.hElevatorGearDiameter * Math.PI/ElevatorConstants.hElevatorEncoderPPR; // distance per encoder pulse
        public static final int hElevatorEncoderTopValue = ;
        public static final double hElevatorEncoderBottomValue = 0;
        public static final double kIVertical = 0;
        public static final double kDVertical = 0;
        public static final double kPVertical = 0;
        public static final double kPHorizontal = 0;
        public static final double kIHorizontal = 0;
        public static final double kDHorizontal = 0;
    
    
    
    } 
    
		  


}
