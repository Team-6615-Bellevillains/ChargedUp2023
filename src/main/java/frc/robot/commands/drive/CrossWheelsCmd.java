package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class CrossWheelsCmd extends CommandBase {

    /*
    * Forms the following shape with the robot's wheels
    *  /                    \
    * /                      \
    *
    * \                      /
    *  \                    /
    * */

    private SwerveSubsystem swerveSubsystem;
    private double initializeTS;
    private final static double convergenceTime = .5;

    private static final Rotation2d frontLeftWheelLockAngle = Rotation2d.fromDegrees(-45);
    private static final Rotation2d frontRightWheelLockAngle = Rotation2d.fromDegrees(45);
    private static final Rotation2d backLeftWheelLockAngle = Rotation2d.fromDegrees(45);
    private static final Rotation2d backRightWheelLockAngle = Rotation2d.fromDegrees(-45);

    public CrossWheelsCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        initializeTS = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        swerveSubsystem.setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, frontLeftWheelLockAngle),
                new SwerveModuleState(0, frontRightWheelLockAngle),
                new SwerveModuleState(0, backLeftWheelLockAngle),
                new SwerveModuleState(0, backRightWheelLockAngle)
        }, false);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Balance Status", "Wheels Locked");

        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > (initializeTS + convergenceTime);
    }

}
