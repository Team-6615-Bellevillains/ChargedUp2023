package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.HorizontalElevatorSubsystem;

public class HorizontalElevatorToSetpointCmd extends CommandBase {

    private final HorizontalElevatorSubsystem horizontalElevatorSubsystem;

    private final ProfiledPIDController profiledPIDController = new ProfiledPIDController(ElevatorConstants.kPHorizontalElevator, ElevatorConstants.kIHorizontalElevator, ElevatorConstants.kDHorizontalElevator, new TrapezoidProfile.Constraints(0, 0));
    private final SimpleMotorFeedforward simpleMotorFeedforward = new SimpleMotorFeedforward(ElevatorConstants.kSHorizontalElevator, ElevatorConstants.kVHorizontalElevator, ElevatorConstants.kAHorizontalElevator);

    public HorizontalElevatorToSetpointCmd(HorizontalElevatorSubsystem horizontalElevatorSubsystem, double setpoint) {
        this.horizontalElevatorSubsystem = horizontalElevatorSubsystem;

        profiledPIDController.setGoal(new TrapezoidProfile.State(setpoint, 0));
    }

    @Override
    public void initialize() {
        profiledPIDController.reset(horizontalElevatorSubsystem.getHorizontalElevatorPosition(), horizontalElevatorSubsystem.getHorizontalElevatorVelocity());
    }

    @Override
    public void execute() {
        double pidOutput = profiledPIDController.calculate(horizontalElevatorSubsystem.getHorizontalElevatorPosition());
        double feedforwardOutput = simpleMotorFeedforward.calculate(profiledPIDController.getSetpoint().velocity);

        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void end(boolean interrupted) {
        horizontalElevatorSubsystem.setHorizontalElevatorVoltage(0); // TODO: Test if this needs to have a static value to keep the arm held up
    }

    @Override
    public boolean isFinished() {
        return profiledPIDController.atGoal();
    }

}
