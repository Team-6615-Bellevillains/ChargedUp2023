package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.TunableSimpleMotorFeedforward;

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

    private double lastKnownCorrectHeadingRadians;
    private final ProfiledPIDController thetaCorrectionPID = new ProfiledPIDController(DriveConstants.kPThetaCorrection,
            DriveConstants.kIThetaCorrection, DriveConstants.kDThetaCorrection, new TrapezoidProfile.Constraints(
                    DriveConstants.kMaxVelocityThetaCorrection, DriveConstants.kMaxAccelerationThetaCorrection));
    private static final TunableSimpleMotorFeedforward driveFeedforward = new TunableSimpleMotorFeedforward("drive",
            0.440000, 2.350000);
    private static final TunableSimpleMotorFeedforward steerFeedforward = new TunableSimpleMotorFeedforward("steer",
            1.112500, 1.300000);
    private double speedMultiplier = 2;
    private double gyroYawOffset = 0;

    public SwerveSubsystem() {
        this.thetaCorrectionPID.enableContinuousInput(0, 2 * Math.PI);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();

                lastKnownCorrectHeadingRadians = getRotation2d().getRadians();
                this.thetaCorrectionPID.reset(lastKnownCorrectHeadingRadians);
            } catch (Exception e) {
                Commands.print("Failed to zero gyro on startup!");
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
        gyroYawOffset = 0;
        System.out.println("Zeroed gyro!");
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d().rotateBy(Rotation2d.fromDegrees(gyroYawOffset));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPoseEstimator(Pose2d pose) {
        gyroYawOffset = pose.getRotation().getDegrees();
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public float getRawRollVelocity() {
        return gyro.getRawGyroY();
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        SmartDashboard.putString("Robot Location",
                getPose().getTranslation().toString());

        SmartDashboard.putNumber("Last known correct heading Rads", lastKnownCorrectHeadingRadians);

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());

        SmartDashboard.putNumber("Speed Multi", getSpeedMultiplier());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public static double calculateDriveFeedforward(double velocity) {
        return driveFeedforward.getController().calculate(velocity);
    }

    public static double calculateSteerFeedforward(double velocity) {
        return steerFeedforward.getController().calculate(velocity);
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

    // NEED TO ADD THE PID CONSTANTS
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        // this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
                                                    // will only use feedforwards.
                        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
                                                    // them 0 will only use feedforwards.
                        (SwerveModuleState[] desiredStates) -> this.setModuleStates(desiredStates, true), // Module
                                                                                                          // states
                                                                                                          // consumer
                        true, // Should the path be automatically mirrored depending on alliance color.
                              // Optional, defaults to true
                        this // Requires this drive subsystem
                ));
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }
}
