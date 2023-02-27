// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToAprilTagCubeCmd extends CommandBase {
  /** Creates a new AlignToMidRungCmd. */
  private LimelightSubsystem limelightSubsystem;
  private SwerveSubsystem swerveSubsystem;

  // These PID controllers will calculate the velocities required to make the
  // robot move to a certain position at a certain angle
  private ProfiledPIDController headingController;
  private ProfiledPIDController xDistanceController;
  private ProfiledPIDController yDistanceController;

  public AlignToAprilTagCubeCmd(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.swerveSubsystem = swerveSubsystem;

    headingController = new ProfiledPIDController(AutoConstants.kPHeadingTrackingDrive, 0, 0, new TrapezoidProfile.Constraints(AutoConstants.kAutoMaxAngularSpeedRadiansPerSecond, AutoConstants.kAutoMaxAngularAccelerationRadiansPerSecondSquared));
    yDistanceController = new ProfiledPIDController(AutoConstants.kPYTrackingDrive, 0, 0, new TrapezoidProfile.Constraints(AutoConstants.kAutoMaxSpeedMetersPerSecond, AutoConstants.kAutoMaxAngularAccelerationRadiansPerSecondSquared));
    xDistanceController = new ProfiledPIDController(AutoConstants.kPXTrackingDrive, 0, 0, new TrapezoidProfile.Constraints(AutoConstants.kAutoMaxSpeedMetersPerSecond, AutoConstants.kAutoMaxAngularAccelerationRadiansPerSecondSquared));

    addRequirements(limelightSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPosition = swerveSubsystem.getPose();
    double currentYPosition = currentPosition.getY();
    double currentXPosition = currentPosition.getX();

    PhotonTrackedTarget bestTarget = limelightSubsystem.getBestTarget();

    if (bestTarget != null) {
      Transform3d cameraTransform = bestTarget.getBestCameraToTarget();

      headingController.setGoal(0);
      yDistanceController.setGoal(currentYPosition + cameraTransform.getY() + 0.331);
      xDistanceController.setGoal(currentXPosition + (cameraTransform.getX() - 1));
    } else {
      this.end(false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPosition = swerveSubsystem.getPose();
    double currentHeading = swerveSubsystem.getHeading();
    double currentYPosition = currentPosition.getY();
    double currentXPosition = currentPosition.getX();

    double rotationOutput = headingController.calculate(currentHeading);
    double ydistanceOutput = yDistanceController.calculate(currentYPosition);
    double xdistanceOutput = xDistanceController.calculate(currentXPosition);

    // Convert P[ID] outputs to ChassisSpeed values, clamping the distance P[ID] to
    // a max speed
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xdistanceOutput, ydistanceOutput, rotationOutput);

    // Convert ChassisSpeeds to SwerveModuleStates and send them off through the
    // SwerveSubsystem
    swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return headingController.atGoal() && yDistanceController.atGoal() && xDistanceController.atGoal();
  }
}
