// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private SwerveSubsystem swerveSubsystem;
  private BangBangController balancer;
  public AutoBalance(SwerveSubsystem swerveSubsystem) {

    this.swerveSubsystem = swerveSubsystem;
    this.balancer = new BangBangController();
    addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //Sets the goal balance of 0 degrees
    balancer.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    double yDistanceOutput = balancer.calculate(swerveSubsystem.getPitch()); 

    // Convert P[ID] outputs to ChassisSpeed values, clamping the distance P[ID] to a max speed
    ChassisSpeeds chassisSpeeds = swerveSubsystem.calculateChassisSpeedsWithDriftCorrection( MathUtil.clamp(0, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
                    AutoConstants.kAutoMaxSpeedMetersPerSecond),MathUtil.clamp(yDistanceOutput, -AutoConstants.kAutoMaxSpeedMetersPerSecond,
                    AutoConstants.kAutoMaxSpeedMetersPerSecond),0, false);

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
    return balancer.atSetpoint();
  }
}
