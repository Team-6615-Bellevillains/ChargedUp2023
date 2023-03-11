package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;
import frc.robot.utils.TunableElevatorFeedforward;
import frc.robot.utils.TunableProfiledPIDController;

public class VerticalElevatorToSetpointCmd extends CommandBase {

    private final VerticalElevatorSubsystem verticalElevatorSubsystem;

    private final TunableProfiledPIDController tunableProfiledPIDController;

    public VerticalElevatorToSetpointCmd(VerticalElevatorSubsystem verticalElevatorSubsystem, double setpointMeters) {
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;

        this.tunableProfiledPIDController = new TunableProfiledPIDController("vertElevator", ElevatorConstants.kPVerticalElevator, ElevatorConstants.kIVerticalElevator, ElevatorConstants.kDVerticalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityVerticalElevator, ElevatorConstants.kMaxAccelerationVerticalElevator), verticalElevatorSubsystem::getVerticalElevatorPosition);

        this.tunableProfiledPIDController.getController().setGoal(setpointMeters);

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        tunableProfiledPIDController.getController().reset(verticalElevatorSubsystem.getVerticalElevatorPosition());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("[VERT] Setpoint TS", Timer.getFPGATimestamp());

        double pidOut = tunableProfiledPIDController.getController().calculate(verticalElevatorSubsystem.getVerticalElevatorPosition());
        SmartDashboard.putNumber("Intermediate setpoint position (meters)", tunableProfiledPIDController.getController().getSetpoint().position);
        SmartDashboard.putNumber("Intermediate setpoint velocity (meters per second)", tunableProfiledPIDController.getController().getSetpoint().velocity);
        double ffOut = verticalElevatorSubsystem.calculateFeedforward(tunableProfiledPIDController.getController().getSetpoint().velocity);

        SmartDashboard.putNumber("PID Out", pidOut);
        SmartDashboard.putNumber("FF Out", ffOut);
        SmartDashboard.putNumber("Theoretical Voltage", pidOut+ffOut);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("[Vert] End Position", verticalElevatorSubsystem.getVerticalElevatorPosition());
        SmartDashboard.putNumber("[Vert] End Velocity", verticalElevatorSubsystem.getVerticalElevatorVelocity());
        SmartDashboard.putNumber("[Vert] End TS", Timer.getFPGATimestamp());
        verticalElevatorSubsystem.setVerticalElevatorVoltage(verticalElevatorSubsystem.calculateFeedforward(0));
    }

    // TODO: Add position tolerance)
//    @Override
//    public boolean isFinished() {
//        return tunableProfiledPIDController.getController().atGoal();
//    }

}
