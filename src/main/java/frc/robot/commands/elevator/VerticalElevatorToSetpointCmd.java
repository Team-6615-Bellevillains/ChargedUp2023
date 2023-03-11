package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.VerticalElevatorSubsystem;
import frc.robot.utils.TunableProfiledPIDController;

public class VerticalElevatorToSetpointCmd extends CommandBase {
    // TODO: Tune 'er up

    private final VerticalElevatorSubsystem verticalElevatorSubsystem;
    private TunableProfiledPIDController tunableProfiledPIDController = new TunableProfiledPIDController("vertElevator", ElevatorConstants.kPVerticalElevator, ElevatorConstants.kIVerticalElevator, ElevatorConstants.kDVerticalElevator, new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityVerticalElevator, ElevatorConstants.kMaxAccelerationVerticalElevator));

    public VerticalElevatorToSetpointCmd(VerticalElevatorSubsystem verticalElevatorSubsystem, double setpoint) {
        tunableProfiledPIDController.getController().setGoal(setpoint);

        this.verticalElevatorSubsystem = verticalElevatorSubsystem;

        addRequirements(verticalElevatorSubsystem);
    }

    @Override
    public void initialize() {
        tunableProfiledPIDController.getController().reset(verticalElevatorSubsystem.getVerticalElevatorPosition(), verticalElevatorSubsystem.getVerticalElevatorVelocity());
    }

    @Override
    public void execute() {
        double pidOutput = tunableProfiledPIDController.calculateAndUpdateLastMeasurement(verticalElevatorSubsystem.getVerticalElevatorPosition());
        double feedforwardOutput = verticalElevatorSubsystem.calculateFeedforward(tunableProfiledPIDController.getController().getSetpoint().velocity);

        SmartDashboard.putNumber("Position Desired (in)", Units.metersToInches(tunableProfiledPIDController.getController().getSetpoint().position));

        verticalElevatorSubsystem.setVerticalElevatorVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorSubsystem.stopElevator();
    }

    @Override
    public boolean isFinished() {
        return tunableProfiledPIDController.getController().atGoal();
    }

}
