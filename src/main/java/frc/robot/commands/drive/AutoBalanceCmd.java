// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCmd extends CommandBase {
  /** Creates a new AutoBalance. */
  private SwerveSubsystem swerveSubsystem;
  private final static double tolerance = 3;
  public AutoBalanceCmd(SwerveSubsystem swerveSubsystem) {

    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            AutoConstants.kBalanceSpeedMetersPerSecond,
            0,
            0,
            swerveSubsystem.getRotation2d()
    );

    // Convert ChassisSpeeds to SwerveModuleStates and send them off through the SwerveSubsystem
    swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swerveSubsystem.getPitch()) <= tolerance;
  }
}
