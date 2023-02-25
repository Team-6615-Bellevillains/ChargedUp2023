package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class VerticalElevatorHighCmd extends CommandBase {

    private final VerticalElevatorToSetpointCmd verticalElevatorToSetpointCmd;

    public VerticalElevatorHighCmd(ElevatorSubsystem elevatorSubsystem) {
        this.verticalElevatorToSetpointCmd = new VerticalElevatorToSetpointCmd(elevatorSubsystem, ElevatorConstants.verticalMaxHeight);

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        verticalElevatorToSetpointCmd.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        verticalElevatorToSetpointCmd.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return verticalElevatorToSetpointCmd.isFinished();
    }

}