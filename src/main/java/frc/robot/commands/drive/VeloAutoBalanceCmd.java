// Based on https://github.com/STMARobotics/frc-7028-2023/blob/f37a74dc3b0cd4d1e4aff6895266caa4fafdb8cb/src/main/java/frc/robot/commands/autonomous/BalanceCommand.java from team 7028
package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class VeloAutoBalanceCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private boolean finished;
    private final float balancingAccelThreshold = -9;
    private double currRoll;

    public VeloAutoBalanceCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Balance Status", "Initialize");
        finished = false;
        currRoll = swerveSubsystem.getRoll();
    }
    
    @Override
    public void execute() {
        double updatedRoll = swerveSubsystem.getRoll();
        double yVelo = (updatedRoll-currRoll)/0.02;
        SmartDashboard.putNumber("Y Velo", yVelo);
        SmartDashboard.putNumber("Curr Roll", currRoll);
        SmartDashboard.putNumber("Updated Roll", updatedRoll);
        SmartDashboard.putNumber("Diff", updatedRoll-currRoll);

        currRoll = updatedRoll;
        if (finished || yVelo < balancingAccelThreshold) {
            SmartDashboard.putString("Balance Status", "Finished Balancing");

            finished = true;
            swerveSubsystem.stopModules();
        } else {
            SmartDashboard.putString("Balance Status", "Balancing");
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.55, 0, 0, swerveSubsystem.getRotation2d());

            swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), false);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
