package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class VerticalElevatorToSetpointCmd extends CommandBase {

    private final VerticalElevatorSubsystem verticalElevatorSubsystem;
    private ProfiledPIDController profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPVerticalElevator, ElevatorConstants.kIVerticalElevator, ElevatorConstants.kDVerticalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityVerticalElevator, ElevatorConstants.kMaxAccelerationVerticalElevator));

    public VerticalElevatorToSetpointCmd(VerticalElevatorSubsystem verticalElevatorSubsystem, double setpoint) {
        profiledPIDController.setGoal(setpoint);

        this.verticalElevatorSubsystem = verticalElevatorSubsystem;

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        profiledPIDController.reset(verticalElevatorSubsystem.getVerticalElevatorPosition(), verticalElevatorSubsystem.getVerticalElevatorVelocity());
    }

    @Override
    public void execute() {
        double pidOutput = profiledPIDController.calculate(verticalElevatorSubsystem.getVerticalElevatorPosition());
        double feedforwardOutput = verticalElevatorSubsystem.calculateFeedforward(profiledPIDController.getSetpoint().velocity);

        verticalElevatorSubsystem.setVerticalElevatorVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return profiledPIDController.atGoal();
    }

}
