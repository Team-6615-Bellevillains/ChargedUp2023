package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroSwerveWheelsCmd extends CommandBase {

    private SwerveSubsystem swerveSubsystem;

    public ZeroSwerveWheelsCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromRadians(0)),
                new SwerveModuleState(0, Rotation2d.fromRadians(0)),
                new SwerveModuleState(0, Rotation2d.fromRadians(0)),
                new SwerveModuleState(0, Rotation2d.fromRadians(0))}, false);
    }

}
