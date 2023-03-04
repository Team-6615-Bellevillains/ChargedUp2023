package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

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

    public SwerveSubsystem() {
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
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // If some wheels are set to a speed over their max speed, they will cap out at
        // their max speed.
        // This messes up the ratio of the wheel speeds to each other.
        // This code will scale down the speeds so they are all within the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    //NEED TO ADD THE PID CONSTANTS
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                   //this.resetOdometry(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 this::getPose, // Pose supplier
                 DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                 new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates, // Module states consumer
                 true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                 this // Requires this drive subsystem
             )
         );
     }
}
