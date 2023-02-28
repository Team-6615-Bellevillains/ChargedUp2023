package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

import java.lang.reflect.Field;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            0,
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftSteerMotorPort,
            DriveConstants.kFrontLeftEncoderAPort,
            DriveConstants.kFrontLeftEncoderBPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetCounts);

    private final SwerveModule frontRight = new SwerveModule(
            1,
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightSteerMotorPort,
            DriveConstants.kFrontRightEncoderAPort,
            DriveConstants.kFrontRightEncoderBPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetCounts);

    private final SwerveModule backLeft = new SwerveModule(
            2,
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftSteerMotorPort,
            DriveConstants.kBackLeftEncoderAPort,
            DriveConstants.kBackLeftEncoderBPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetCounts);

    private final SwerveModule backRight = new SwerveModule(
            3,
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightSteerMotorPort,
            DriveConstants.kBackRightEncoderAPort,
            DriveConstants.kBackRightEncoderBPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetCounts);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            getRotation2d(), getModulePositions(), new Pose2d());

    private final static NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private final static NetworkTable tuningTable = networkTableInstance.getTable("tuning");
    private double lastUpdatedTS = Timer.getFPGATimestamp();

    public SwerveSubsystem() {
        tuningTable.getEntry("kPTurning").setDouble(SwerveModuleConstants.kPTurning);
        tuningTable.getEntry("kITurning").setDouble(SwerveModuleConstants.kITurning);
        tuningTable.getEntry("kDTurning").setDouble(SwerveModuleConstants.kDTurning);
        tuningTable.getEntry("maxWheelVelocity").setDouble(SwerveModuleConstants.maxWheelVelocity);
        tuningTable.getEntry("maxWheelAcceleration").setDouble(SwerveModuleConstants.maxWheelAcceleration);

        tuningTable.getEntry("kSTurning").setDouble(SwerveModuleConstants.kSTurning);
        tuningTable.getEntry("kVTurning").setDouble(SwerveModuleConstants.kVTurning);
        tuningTable.getEntry("kATurning").setDouble(SwerveModuleConstants.kATurning);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
    }

    public void zeroHeading() {
        System.out.println("Zeroing gyro!");
        gyro.reset();
        System.out.println("Zeroed gyro!");
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location",
                getPose().getTranslation().toString());


        double nowTime = Timer.getFPGATimestamp();
        if (nowTime - lastUpdatedTS >= 5) {
            SimpleMotorFeedforward flFeedforward = frontLeft.getSteerFeedforward();
            ProfiledPIDController flProfiledPIDController = frontLeft.getSteerPIDController();

            double tableKP = tuningTable.getValue("kPTurning").getDouble();
            double tableKI = tuningTable.getValue("kITurning").getDouble();
            double tableKD = tuningTable.getValue("kDTurning").getDouble();
            double tableMaxWheelVelocity = tuningTable.getValue("maxWheelVelocity").getDouble();
            double tableMaxWheelAcceleration = tuningTable.getValue("maxWheelAcceleration").getDouble();

            TrapezoidProfile.Constraints flProfilePIDControllerConstraints;
            try {
                Field constraintsField = ProfiledPIDController.class.getDeclaredField("m_constraints");
                constraintsField.setAccessible(true);
                flProfilePIDControllerConstraints = (TrapezoidProfile.Constraints) constraintsField.get(flProfiledPIDController);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                throw new RuntimeException(e);
            }

            double tableKS = tuningTable.getValue("kSTurning").getDouble();
            double tableKV = tuningTable.getValue("kVTurning").getDouble();
            double tableKA = tuningTable.getValue("kATurning").getDouble();

            if (flProfiledPIDController.getP() != tableKP
                    || flProfiledPIDController.getI() != tableKI
                    || flProfiledPIDController.getD() != tableKD
                    || flProfilePIDControllerConstraints.maxVelocity != tableMaxWheelVelocity
                    || flProfilePIDControllerConstraints.maxAcceleration != tableMaxWheelAcceleration) {
                frontLeft.changePIDConstants(tableKP, tableKI, tableKD, tableMaxWheelVelocity, tableMaxWheelAcceleration);
                frontRight.changePIDConstants(tableKP, tableKI, tableKD, tableMaxWheelVelocity, tableMaxWheelAcceleration);
                backLeft.changePIDConstants(tableKP, tableKI, tableKD, tableMaxWheelVelocity, tableMaxWheelAcceleration);
                backRight.changePIDConstants(tableKP, tableKI, tableKD, tableMaxWheelVelocity, tableMaxWheelAcceleration);
            }

            if (flFeedforward.ks != tableKS || flFeedforward.kv != tableKV || flFeedforward.ka != tableKA) {
                frontLeft.changeFeedforwardConstants(tableKS, tableKV, tableKA);
                frontRight.changeFeedforwardConstants(tableKS, tableKV, tableKA);
                backLeft.changeFeedforwardConstants(tableKS, tableKV, tableKA);
                backRight.changeFeedforwardConstants(tableKS, tableKV, tableKA);
            }

            lastUpdatedTS = nowTime;
        }
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean ignoreLittle) {
        // If some wheels are set to a speed over their max speed, they will cap out at
        // their max speed.
        // This messes up the ratio of the wheel speeds to each other.
        // This code will scale down the speeds so they are all within the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0], ignoreLittle);
        frontRight.setDesiredState(desiredStates[1], ignoreLittle);
        backLeft.setDesiredState(desiredStates[2], ignoreLittle);
        backRight.setDesiredState(desiredStates[3], ignoreLittle);
    }
}
