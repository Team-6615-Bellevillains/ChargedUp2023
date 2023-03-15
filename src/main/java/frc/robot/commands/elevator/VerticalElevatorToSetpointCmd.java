package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class VerticalElevatorToSetpointCmd extends CommandBase {

    private final VerticalElevatorSubsystem verticalElevatorSubsystem;

    private final ProfiledPIDController profiledPIDController;

    public VerticalElevatorToSetpointCmd(VerticalElevatorSubsystem verticalElevatorSubsystem, double setpointMeters) {
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;

        this.profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPVerticalElevator, ElevatorConstants.kIVerticalElevator, ElevatorConstants.kDVerticalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityVerticalElevator, ElevatorConstants.kMaxAccelerationVerticalElevator));

        this.profiledPIDController.setGoal(setpointMeters);

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        profiledPIDController.reset(verticalElevatorSubsystem.getVerticalElevatorPosition());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("[VERT] Setpoint TS", Timer.getFPGATimestamp());
        SmartDashboard.putNumber("[VERT] Setpoint Goal Position", profiledPIDController.getGoal().position);


        double pidOut = profiledPIDController.calculate(verticalElevatorSubsystem.getVerticalElevatorPosition());
        SmartDashboard.putNumber("[VERT] Intermediate setpoint position (meters)", profiledPIDController.getSetpoint().position);
        SmartDashboard.putNumber("[VERT] Intermediate setpoint velocity (meters per second)", profiledPIDController.getSetpoint().velocity);
        double ffOut = verticalElevatorSubsystem.calculateFeedforward(profiledPIDController.getSetpoint().velocity);

        SmartDashboard.putNumber("[VERT] PID Out", pidOut);
        SmartDashboard.putNumber("[VERT] FF Out", ffOut);
        SmartDashboard.putNumber("[VERT] Theoretical Voltage", pidOut+ffOut);
        SmartDashboard.putNumber("[VERT] Error", Math.abs(verticalElevatorSubsystem.getVerticalElevatorPosition()-profiledPIDController.getGoal().position));

        verticalElevatorSubsystem.setVerticalElevatorVoltage(pidOut+ffOut);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("[VERT] End Position", verticalElevatorSubsystem.getVerticalElevatorPosition());
        SmartDashboard.putNumber("[VERT] End Velocity", verticalElevatorSubsystem.getVerticalElevatorVelocity());
        SmartDashboard.putNumber("[VERT] End TS", Timer.getFPGATimestamp());
        verticalElevatorSubsystem.stopElevator();
    }

    // TODO: Add position tolerance
    @Override
    public boolean isFinished() {
        return Math.abs(verticalElevatorSubsystem.getVerticalElevatorPosition()-profiledPIDController.getGoal().position) <= Units.inchesToMeters(.25);
    }

}
