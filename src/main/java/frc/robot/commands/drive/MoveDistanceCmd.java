package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveDistanceCmd extends CommandBase {

    private SwerveSubsystem swerveSubsystem;

    private double xDistance, yDistance;
    private double finalXMeters, finalYMeters;

    private ProfiledPIDController xProfiledPIDController;
    private ProfiledPIDController yProfiledPIDController;

    public MoveDistanceCmd(SwerveSubsystem swerveSubsystem, double xDistance, double yDistance) {
        this.swerveSubsystem = swerveSubsystem;

        this.xProfiledPIDController = new ProfiledPIDController(
                DriveConstants.kPMoveXDistance,
                DriveConstants.kIMoveXDistance,
                DriveConstants.kDMoveXDistance,
                new TrapezoidProfile.Constraints(
                        AutoConstants.kAutoMaxSpeedMetersPerSecond,
                        AutoConstants.kAutoMaxAccelerationMetersPerSecond));

        this.yProfiledPIDController = new ProfiledPIDController(
                DriveConstants.kPMoveYDistance,
                DriveConstants.kIMoveYDistance,
                DriveConstants.kDMoveYDistance,
                new TrapezoidProfile.Constraints(
                        AutoConstants.kAutoMaxSpeedMetersPerSecond,
                        AutoConstants.kAutoMaxAccelerationMetersPerSecond));

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currPose = swerveSubsystem.getPose();
        // TODO: Use a translated pose instead
        this.finalXMeters = currPose.getX() + xDistance;
        this.finalYMeters = currPose.getY() + yDistance;

        xProfiledPIDController.reset(new TrapezoidProfile.State(0, 0));
        yProfiledPIDController.reset(new TrapezoidProfile.State(0, 0));
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();

        double xSpeed = this.xProfiledPIDController.calculate(currentPose.getX(), finalXMeters);
        double ySpeed = this.yProfiledPIDController.calculate(currentPose.getY(), finalYMeters);

        swerveSubsystem.setModuleStates(
                DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, 0)));
    }

    @Override
    public boolean isFinished() {
        return xProfiledPIDController.atSetpoint() && yProfiledPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

}
