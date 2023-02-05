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

public class AlignToAprilTag extends CommandBase {

    private LimelightSubsystem limelightSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private PIDController yawController;
    private PIDController distanceController;

    public AlignToAprilTag(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        yawController = new PIDController(.05, 0, 0);
        distanceController = new PIDController(.7, 0, 0);

        addRequirements(limelightSubsystem);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = limelightSubsystem.getBestTarget();

        if (target != null) {
            double yaw = target.getYaw();
            Transform3d cameraTransform = target.getBestCameraToTarget();
            double distanceToTarget = cameraTransform.getTranslation().getX();

            double rotationOutput = yawController.calculate(yaw, 0);
            double distanceOutput = distanceController.calculate(distanceToTarget, .45);

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                    -MathUtil.clamp(distanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
                            AutoConstants.kAutoMaxSpeedMetersPerSecond),
                    0,
                    rotationOutput);

            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        } else {
            swerveSubsystem.stopModules();
        }
    }

}
