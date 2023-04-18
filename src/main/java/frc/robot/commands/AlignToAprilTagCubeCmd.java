// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.TunablePIDController;

public class AlignToAprilTagCubeCmd extends CommandBase {
    /** Creates a new AlignToMidRungCmd. */
    private LimelightSubsystem limelightSubsystem;
    private SwerveSubsystem swerveSubsystem;

    // These PID controllers will calculate the velocities required to make the
    // robot move to a certain position at a certain angle
    private PIDController xdistanceController;
    private PIDController ydistanceController;
    private double ySetpoint;
    private double xSetpoint;
    private PhotonTrackedTarget target;
    private Pose2d currentPosition;
    private double currentYPosition;
    private double currentXPosition;

    public AlignToAprilTagCubeCmd(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        // yawController = new PIDController(AutoConstants.kPTrackingYaw, 0, 0);
        xdistanceController = new TunablePIDController("xdistance", 1.2, 0, 0).getController();
        ydistanceController = new TunablePIDController("ydistance", 1.2, 0, 0).getController();
        ySetpoint = 0;
        xSetpoint = 0;

        addRequirements(limelightSubsystem, swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xdistanceController.reset();
        ydistanceController.reset();
        currentPosition = swerveSubsystem.getPose();
        currentYPosition = currentPosition.getY();
        currentXPosition = currentPosition.getX();

        xdistanceController.reset();
        ydistanceController.reset();

        if (limelightSubsystem.getBestTarget() != null) {
            target = limelightSubsystem.getBestTarget();
            Transform3d cameraTransform = target.getBestCameraToTarget();

            ySetpoint = currentYPosition + cameraTransform.getY() + 0.331 - Units.inchesToMeters(5);

            // xSetpoint = currentXPosition + (cameraTransform.getX() - 1 +
            // Units.inchesToMeters(9));
            xSetpoint = (currentXPosition + cameraTransform.getX() + 0.07)
                    - (Constants.LimelightConstants.bumperOffset + Constants.LimelightConstants.xCameraOffset + 0.400);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentPosition = swerveSubsystem.getPose();
        currentYPosition = currentPosition.getY();
        currentXPosition = currentPosition.getX();

        // double rotationOutput =
        // yawController.calculate(limelightSubsystem.getBestTarget().getYaw(), 0);
        double ydistanceOutput = ydistanceController.calculate(currentYPosition, ySetpoint);
        double xdistanceOutput = xdistanceController.calculate(currentXPosition, xSetpoint);

        /*
         * SmartDashboard.putNumber("y curr", currentYPosition);
         * SmartDashboard.putNumber("x curr", currentXPosition);
         * // SmartDashboard.putNumber("x P", xdistanceController.getP());
         * SmartDashboard.putNumber("x goal", xSetpoint);
         * SmartDashboard.putNumber("y goal", ySetpoint);
         */
        // Convert P[ID] outputs to ChassisSpeed values, clamping the distance P[ID] to
        // a max speed

        /*
         * ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
         * MathUtil.clamp(xdistanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
         * AutoConstants.kAutoMaxSpeedMetersPerSecond),
         * MathUtil.clamp(ydistanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
         * AutoConstants.kAutoMaxSpeedMetersPerSecond),
         * 0);
         */

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xdistanceOutput, ydistanceOutput, 0);

        // Convert ChassisSpeeds to SwerveModuleStates and send them off through the
        // SwerveSubsystem
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), true);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putNumber("Align end TS", Timer.getFPGATimestamp());
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xdistanceController.atSetpoint() && ydistanceController.atSetpoint();
    }
}
