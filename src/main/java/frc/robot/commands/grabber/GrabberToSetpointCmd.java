package frc.robot.commands.grabber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GrabberConstants;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.utils.TunableProfiledPIDController;

public class GrabberToSetpointCmd extends CommandBase {

    private final GrabberSubsystem grabberSubsystem;

    private final TunableProfiledPIDController profiledPIDController;

    public GrabberToSetpointCmd(GrabberSubsystem grabberSubsystem, double setpointRadians) {
        this.grabberSubsystem = grabberSubsystem;

        this.profiledPIDController = new TunableProfiledPIDController("grabber", GrabberConstants.kPFlip,
                GrabberConstants.kIFlip, GrabberConstants.kDFlip,
                new TrapezoidProfile.Constraints(GrabberConstants.kMaxFlipVelocityRadiansPerSecond,
                        GrabberConstants.kMaxFlipAccelerationRadiansPerSecondSquared),
                grabberSubsystem::getFlipEncoderPositionInRads);

        this.profiledPIDController.getController().setGoal(setpointRadians);

        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        profiledPIDController.getController().reset(grabberSubsystem.getFlipEncoderPositionInRads());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("[GRAB] Setpoint TS", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("[GRAB] Setpoint Goal Position",
                profiledPIDController.getController().getGoal().position);

        double pidOut = profiledPIDController.getController()
                .calculate(grabberSubsystem.getFlipEncoderPositionInRads());
        SmartDashboard.putNumber("[GRAB] Intermediate setpoint position (rads)",
                profiledPIDController.getController().getSetpoint().position);
        SmartDashboard.putNumber("[GRAB] Intermediate setpoint velocity (rads per second)",
                profiledPIDController.getController().getSetpoint().velocity);
        double ffOut = grabberSubsystem.calculateFeedforward(grabberSubsystem.getFlipEncoderPositionInRads(),
                profiledPIDController.getController().getSetpoint().velocity);

        SmartDashboard.putNumber("[GRAB] PID Out", pidOut);
        SmartDashboard.putNumber("[GRAB] FF Out", ffOut);
        SmartDashboard.putNumber("[GRAB] Theoretical Voltage", pidOut + ffOut);

        grabberSubsystem.setMotorVoltage(pidOut + ffOut);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("[GRAB] End Position", grabberSubsystem.getFlipEncoderPositionInRads());
        SmartDashboard.putNumber("[GRAB] End Velocity", grabberSubsystem.getFlipEncoderVelocityInRadsPerSec());
        SmartDashboard.putNumber("[GRAB] End TS", Timer.getFPGATimestamp());
        grabberSubsystem.setMotorVoltage(
                grabberSubsystem.calculateFeedforward(grabberSubsystem.getFlipEncoderPositionInRads(), 0));
    }

    // TODO: Add position tolerance
    @Override
    public boolean isFinished() {
        return Math.abs(grabberSubsystem.getFlipEncoderPositionInRads()
                - profiledPIDController.getController().getGoal().position) <= Units.degreesToRadians(6);
    }

}
