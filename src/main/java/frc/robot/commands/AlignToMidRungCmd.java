// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToMidRungCmd extends CommandBase {
  /** Creates a new AlignToMidRungCmd. */
  private LimelightSubsystem limelightSubsystem;
  private SwerveSubsystem swerveSubsystem;

  // These PID controllers will calculate the velocities required to make the
  // robot move to a certain position at a certain angle
  private PIDController yawController;
  private PIDController xdistanceController;
  private PIDController ydistanceController;
  private double setpoint;
  private double xsetpoint;
  private PhotonTrackedTarget target;
  private Pose2d currentPosition;
  private double currentYPosition;
  private double currentXPosition;

  public AlignToMidRungCmd(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.swerveSubsystem = swerveSubsystem;

    //yawController = new PIDController(AutoConstants.kPTrackingYaw, 0, 0);
    xdistanceController = new PIDController(AutoConstants.kPTrackingDrive, 0, 0);
    ydistanceController = new PIDController(AutoConstants.kPTrackingDriveY, 0, 0);
    target = limelightSubsystem.getBestTarget();
    setpoint = 0;
    xsetpoint = 0;

    currentPosition = swerveSubsystem.getPose();
    currentYPosition = currentPosition.getY();
    currentXPosition = currentPosition.getX();
    addRequirements(limelightSubsystem, swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limelightSubsystem.getBestTarget() != null) {
      Transform3d cameraTransform = target.getBestCameraToTarget();

      setpoint = currentYPosition + (-(LimelightConstants.distanceFromAprilTagToRung) + cameraTransform.getY()) + (0.331 - Units.inchesToMeters(17));
      xsetpoint = currentXPosition + (cameraTransform.getX() - 1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = swerveSubsystem.getPose();
    currentYPosition = currentPosition.getY();

    //double rotationOutput = yawController.calculate(limelightSubsystem.getBestTarget().getYaw(), 0);
    double ydistanceOutput = ydistanceController.calculate(currentYPosition, setpoint);
    double xdistanceOutput = xdistanceController.calculate(currentXPosition, xsetpoint);

    // Convert P[ID] outputs to ChassisSpeed values, clamping the distance P[ID] to
    // a max speed
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds( MathUtil.clamp(xdistanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
    AutoConstants.kAutoMaxSpeedMetersPerSecond),
        MathUtil.clamp(ydistanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
            AutoConstants.kAutoMaxSpeedMetersPerSecond),
        0);

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
    return ydistanceController.atSetpoint() && xdistanceController.atSetpoint();
  }
}
