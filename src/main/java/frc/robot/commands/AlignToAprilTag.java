package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToAprilTag extends CommandBase {

    private LimelightSubsystem limelightSubsystem;
    private SwerveSubsystem swerveSubsystem;

    private PIDController yawController;

    public AlignToAprilTag(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        yawController = new PIDController(.01, 0, 0);

        addRequirements(limelightSubsystem);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = limelightSubsystem.getBestTarget();

        if (target != null) {
            double yaw = target.getYaw();

            double rotationOutput = yawController.calculate(yaw, 0);

            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotationOutput);

            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        }
    }

}
