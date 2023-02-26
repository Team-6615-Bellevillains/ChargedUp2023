package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;

public class VerticalElevatorToSetpointCmd extends CommandBase {

    private final VerticalElevatorSubsystem verticalElevatorSubsystem;

    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPVerticalElevator, ElevatorConstants.kIVerticalElevator, ElevatorConstants.kDVerticalElevator, new TrapezoidProfile.Constraints(0, 0));
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kSVerticalElevator, ElevatorConstants.kGVerticalElevator, ElevatorConstants.kVVerticalElevator, ElevatorConstants.kAVerticalElevator);

    public VerticalElevatorToSetpointCmd(VerticalElevatorSubsystem verticalElevatorSubsystem, double setpoint) {
        this.verticalElevatorSubsystem = verticalElevatorSubsystem;

        profiledPIDController.setGoal(new TrapezoidProfile.State(setpoint, 0));
    }

    @Override
    public void initialize() {
        profiledPIDController.reset(verticalElevatorSubsystem.getVerticalElevatorPosition(), verticalElevatorSubsystem.getVerticalElevatorVelocity());
    }

    @Override
    public void execute() {
        double pidOutput = profiledPIDController.calculate(verticalElevatorSubsystem.getVerticalElevatorPosition());
        double feedforwardOutput = elevatorFeedforward.calculate(profiledPIDController.getSetpoint().velocity);

        verticalElevatorSubsystem.setVerticalElevatorVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorSubsystem.setVerticalElevatorVoltage(0); // TODO: Test if this needs to have a static value to keep the arm held up
    }

    @Override
    public boolean isFinished() {
        return profiledPIDController.atGoal();
    }

}
