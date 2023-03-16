package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TipCmd extends CommandBase {

    private SwerveSubsystem swerveSubsystem;

    public TipCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                AutoConstants.kTipSpeedMetersPerSecond,
                0,
                0,
                swerveSubsystem.getRotation2d()
        );

        // Convert ChassisSpeeds to SwerveModuleStates and send them off through the SwerveSubsystem
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), true);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Tip End TS", Timer.getFPGATimestamp());
        SmartDashboard.putBoolean("Tip End Reason", interrupted);
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.getPitch() > 5;
    }

}
