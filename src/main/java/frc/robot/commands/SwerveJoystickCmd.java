package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, steerSpeedFunction;
    private final Supplier<Boolean> isFieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, steerLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> steerSpeedFunction,
            Supplier<Boolean> isFieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.steerSpeedFunction = steerSpeedFunction;
        this.isFieldOrientedFunction = isFieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleOpMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleOpMaxAccelerationUnitsPerSecond);
        this.steerLimiter = new SlewRateLimiter(DriveConstants.kTeleOpMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get joystick inputs
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double steerSpeed = steerSpeedFunction.get();

        /*
         * 2. Apply deadband
         * Our joysticks do not read 0 even when they are completely centered.
         * To account for this, we apply what's known as a deadband.
         * In our case, we take a number and for every number 0 to that number, we map
         * the input to 0.
         */
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        steerSpeed = Math.abs(steerSpeed) > OIConstants.kDeadband ? steerSpeed : 0.0;

        // 3. Smooth driving
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleOpMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleOpMaxSpeedMetersPerSecond;
        steerSpeed = steerLimiter.calculate(steerSpeed) * DriveConstants.kTeleOpMaxAngularSpeedRadiansPerSecond;

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("steerSpeed", steerSpeed);

        /*
         * 4. Calculate ChassisSpeeds
         * WPILib does all the heavy lifting with our kinematics. This ChassisSpeeds
         * object represents a universal container for linear and angular velocities
         * (strafing and rotation).
         */
        ChassisSpeeds chassisSpeeds;
        if (isFieldOrientedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,
                    steerSpeed,
                    swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, steerSpeed);
        }

        /*
         * 5. Convert ChassisSpeeds to SwerveModuleStates
         * For a swerve drive, the ChassisSpeeds needs to be converted into a separate
         * linear and angular velocity for each swerve module.
         */
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
