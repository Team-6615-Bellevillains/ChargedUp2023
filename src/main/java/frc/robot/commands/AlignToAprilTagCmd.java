package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAprilTagCmd extends CommandBase {

    private LimelightSubsystem limelightSubsystem;
    private SwerveSubsystem swerveSubsystem;

    // These PID controllers will calculate the velocities required to make the
    // robot move to a certain position at a certain angle
    private PIDController yawController;
    private PIDController distanceController;

    public AlignToAprilTagCmd(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        yawController = new PIDController(AutoConstants.kPTrackingYaw, 0, 0);
        distanceController = new PIDController(AutoConstants.kPTrackingDrive, 0, 0);

        addRequirements(limelightSubsystem);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = limelightSubsystem.getBestTarget();

        if (target != null) {
            // Get target position (relative to robot) and yaw rotation. We will use these
            // values in the P[ID] loops
            double yaw = target.getYaw();
            Transform3d cameraTransform = target.getBestCameraToTarget();
            double distanceToTarget = cameraTransform.getTranslation().getX();

            // Calculate velocities required to align the robot with the tracking target and
            // move it kTrackingDistance away.
            double rotationOutput = yawController.calculate(yaw, 0);
            double distanceOutput = distanceController.calculate(distanceToTarget, AutoConstants.kTrackingDistance);

            // Convert P[ID] outputs to ChassisSpeed values, clamping the distance P[ID] to
            // a max speed
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                    -MathUtil.clamp(distanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
                            AutoConstants.kAutoMaxSpeedMetersPerSecond),
                    0,
                    rotationOutput);

            // Convert ChassisSpeeds to SwerveModuleStates and send them off through the
            // SwerveSubsystem
            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        } else {
            // If there is no target to track, stop. This prevents the robot from continuing
            // to move after the target is lost.
            swerveSubsystem.stopModules();
        }
    }

}
