// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private SwerveSubsystem swerveSubsystem;
  private PIDController balancePID = new PIDController(0.10, 0, 0);
  private CrossWheelsCmd crossWheelsCmd;  
  public AutoBalance(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    balancePID.setSetpoint(0);
    this.crossWheelsCmd = new CrossWheelsCmd(swerveSubsystem);
    addRequirements(swerveSubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    balancePID.reset();
    balancePID.setTolerance(1.5);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double pitch = swerveSubsystem.getPitch();
    double yDistanceOutput = balancePID.calculate(pitch);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
       0,
        MathUtil.clamp(yDistanceOutput, -0.8,
            0.8),
        0);

    // Convert ChassisSpeeds to SwerveModuleStates and send them off through the
    // SwerveSubsystem
    swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), true);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    crossWheelsCmd.execute();
    swerveSubsystem.stopModules();

    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return balancePID.atSetpoint();
  }
}
