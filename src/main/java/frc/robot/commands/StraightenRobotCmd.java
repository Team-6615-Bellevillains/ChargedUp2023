package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class StraightenRobotCmd extends CommandBase {

    private SwerveSubsystem swerveSubsystem;
    private PIDController rotationPID;
    private SlewRateLimiter rotationLimiter;

    public StraightenRobotCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.rotationLimiter = new SlewRateLimiter(DriveConstants.kTeleOpMaxAngularAccelerationUnitsPerSecond);

        this.rotationPID = new PIDController(DriveConstants.kPRotation, DriveConstants.kIRotation,
                DriveConstants.kDRotation);
        this.rotationPID.enableContinuousInput(-180, 180);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double pidOutput = rotationPID.calculate(swerveSubsystem.getHeading(), 0);
        double rotationSpeed = -this.rotationLimiter.calculate(MathUtil.clamp(pidOutput, -1, 1))
                * DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2;

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotationSpeed);

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

}
